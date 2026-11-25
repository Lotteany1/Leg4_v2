/**
 * @file  tinympc14.c
 * @brief 融合 MPC 单文件实现：核心 内部接口 ADMM 异步 输出层装配
 * @version  v1.8  2026-11-26
 *
 * 分四段：core 求解核心，admm ADMM 递推，async 异步求解与结果发布，output 输出层装配。
 * 输出层三笔补偿的物理量来自固件数学库的 Leg_Comp / Leg_SpringF 与本文件里的表常量。
 */

#include "tinympc14.h"
#include "Leg_Comp.h"
#include "Leg_SpringF.h"

#if defined(TINYMPC14_ONLINE_IFACE_VER)
/* The online deployment appends the generated scheduler to this translation unit. */
void tinympc14_online_model(double left, double right, double psi, double roll_height,
                          double Ad[196], double Bd[84], double fd[14], double P[196]);
#endif

#include <math.h>
#include <string.h>

#define NX TINYMPC14_NX
#define NU TINYMPC14_NU
#define NH TINYMPC14_NH
#define NN TINYMPC14_NN

/* ---- 内部接口 ---- */

void Tinympc14_Affine(const float xref[TINYMPC14_NX],
                         const float fed[TINYMPC14_NX],
                         const float ffwd[TINYMPC14_NU],
                         float u_ss[TINYMPC14_NU],
                         float u_ff[TINYMPC14_NU]);
void Tinympc14_Affine_Pub(const float xref[TINYMPC14_NX],
                             const float fed[TINYMPC14_NX],
                             const float ffwd[TINYMPC14_NU],
                             float u_ss[TINYMPC14_NU],
                             float u_ff[TINYMPC14_NU]);
void Tinympc14_Async_Drop(void);
void Tinympc14_Async_Reset(void);
void Tinympc14_Admm_Reset(void);
int Tinympc14_Admm_Model(const tinympc14_model_t *m);
#if defined(TINYMPC14_ONLINE_IFACE_VER)
static int Tinympc14_Admm_Reference_Model(const tinympc14_model_t *m);
#endif
static int Tinympc14_Admm_Solve(const tinympc14_model_t *m,const float *x0,const float *ref,
                   const float *f,const float *first_f,const float *prev,const float *lo,const float *hi,float *u);
const float *Tinympc14_Admm_Sequence(void);

/* ---- core ---- */

volatile tinympc14_status_t   tinympc14_status;
volatile tinympc14_counters_t tinympc14_counters;
volatile float tinympc14_u[TINYMPC14_NU];

static tinympc14_model_t tinympc14_work;
static double tinympc14_bd_pinv[TINYMPC14_NU * TINYMPC14_NX];

/* 控制拍只读快照：求解任务写另一块，写完好再切索引；控制拍全程只读自己那一块 */
static tinympc14_model_t tinympc14_pub[2];
static double tinympc14_pub_pinv[2][TINYMPC14_NU * TINYMPC14_NX];
#if TINYMPC14_CLOSED_CHAIN_AXIAL_INPUT
static float tinympc14_pub_roll_scale[2]; /* 力矩转成轴向差力时使用同一发布工作点 */
#endif
static volatile int tinympc14_pub_idx;

/* 上次实际烘焙的请求值（未钳位）：钳位区内几何变化仍触发重算 */
static float tinympc14_last_raw[2];
static float tinympc14_last_raw_psi;
static float tinympc14_last_roll_height;
static int   tinympc14_model_ok;

static float tinympc14_fx[TINYMPC14_NX];
static float tinympc14_aff[TINYMPC14_NX];
static float tinympc14_aff_pub[TINYMPC14_NX];
static float tinympc14_fx_pub[TINYMPC14_NX];
static double tinympc14_pinv_a[TINYMPC14_NU][TINYMPC14_NU + TINYMPC14_NX];

#if defined(TINYMPC14_ONLINE_IFACE_VER)
/* Single model-update task: reuse double output buffers across updates. */
static double tinympc14_online_ad[NX * NX];
static double tinympc14_online_bd[NX * NU];
static double tinympc14_online_fd[NX];
static double tinympc14_online_p[NX * NX];
static void Tinympc14_Online_Aux(tinympc14_model_t *model);
#endif

static void Tinympc14_Mul(float *y, const float *a, const float *x, int rows, int cols)
{
    int i, j;

    for (i = 0; i < rows; i++)
    {
        const float *row = a + i * cols;
        double acc = 0.0;

        for (j = 0; j < cols; j++)
        {
            acc += (double)row[j] * x[j];
        }

        y[i] = (float)acc;
    }
}

static int Tinympc14_Pinv(const tinympc14_model_t *m, double *pinv)
{
    int i, j, k, col, r;
    double scale = 0.0;
    double sc[TINYMPC14_NU];
    double (*a)[TINYMPC14_NU + TINYMPC14_NX] = tinympc14_pinv_a;

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        double mx = 0.0;

        for (k = 0; k < TINYMPC14_NX; k++)
        {
            double v = fabs((double)m->Bd[k * TINYMPC14_NU + i]);

            if (v > mx)
            {
                mx = v;
            }
        }

        if (!(mx > 0.0) || !isfinite(mx))
        {
            tinympc14_counters.pinv_degenerate++;
            return -1;
        }

        sc[i] = 1.0 / mx;
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        for (j = 0; j < TINYMPC14_NU; j++)
        {
            double s = 0.0;

            for (k = 0; k < TINYMPC14_NX; k++)
            {
                s += ((double)m->Bd[k * TINYMPC14_NU + i] * sc[i])
                     * ((double)m->Bd[k * TINYMPC14_NU + j] * sc[j]);
            }

            a[i][j] = s;

            if (i == j && fabs(s) > scale)
            {
                scale = fabs(s);
            }
        }
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        for (j = 0; j < TINYMPC14_NX; j++)
        {
            a[i][TINYMPC14_NU + j] = (double)m->Bd[j * TINYMPC14_NU + i] * sc[i];
        }
    }

    if (!(scale > 0.0) || !isfinite(scale))
    {
        tinympc14_counters.pinv_degenerate++;
        return -1;
    }

    for (col = 0; col < TINYMPC14_NU; col++)
    {
        int piv = col;
        double best = fabs(a[col][col]);

        for (r = col + 1; r < TINYMPC14_NU; r++)
        {
            double v = fabs(a[r][col]);

            if (v > best)
            {
                best = v;
                piv = r;
            }
        }

        if (best <= scale * 1e-12)
        {
            tinympc14_counters.pinv_degenerate++;
            return -1;
        }

        if (piv != col)
        {
            for (j = col; j < TINYMPC14_NU + TINYMPC14_NX; j++)
            {
                double t = a[col][j];

                a[col][j] = a[piv][j];
                a[piv][j] = t;
            }
        }

        {
            double pv = a[col][col];

            for (j = col; j < TINYMPC14_NU + TINYMPC14_NX; j++)
            {
                a[col][j] /= pv;
            }
        }

        for (r = 0; r < TINYMPC14_NU; r++)
        {
            double f;

            if (r == col)
            {
                continue;
            }

            f = a[r][col];

            if (f == 0.0)
            {
                continue;
            }

            for (j = col; j < TINYMPC14_NU + TINYMPC14_NX; j++)
            {
                a[r][j] -= f * a[col][j];
            }
        }
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        for (j = 0; j < TINYMPC14_NX; j++)
        {
            pinv[i * TINYMPC14_NX + j] = sc[i] * a[i][TINYMPC14_NU + j];
        }
    }

    return 0;
}

static float Tinympc14_Clamp_Len(float len)
{
    if (!isfinite(len))
    {
        len = TINYMPC14_SCHED_L;
    }

    return Tinympc14_Clip(len, TINYMPC14_L_MIN, TINYMPC14_L_MAX);
}

static float Tinympc14_Clamp_Psi(float psi)
{
    if (!isfinite(psi))
    {
        psi = 0.0f;
    }

    return Tinympc14_Clip(psi, TINYMPC14_LPV_PSI_MIN, TINYMPC14_LPV_PSI_MAX);
}

static int Tinympc14_Model_Force(float lL, float lR, float psi, float roll_height)
{
    int back;
#if defined(TINYMPC14_ONLINE_IFACE_VER)
    int i;

    tinympc14_online_model(lL, lR, psi, roll_height, tinympc14_online_ad, tinympc14_online_bd,
                            tinympc14_online_fd, tinympc14_online_p);
    for (i = 0; i < NX * NX; i++)
    {
        tinympc14_work.Ad[i] = (float)tinympc14_online_ad[i];
        tinympc14_work.Pinf[i] = (float)tinympc14_online_p[i];
    }
    for (i = 0; i < NX * NU; i++) tinympc14_work.Bd[i] = (float)tinympc14_online_bd[i];
    for (i = 0; i < NX; i++) tinympc14_work.fd[i] = (float)tinympc14_online_fd[i];
#else
    Tinympc14_Lpv_Data_Update_Height(lL, lR, psi, roll_height, &tinympc14_work);
#endif

    if (Tinympc14_Pinv(&tinympc14_work, tinympc14_bd_pinv) != 0)
    {
        return -1;
    }

#if defined(TINYMPC14_ONLINE_IFACE_VER)
    /* Preserve the original fallback gain with one terminal-stage factorization.
     * Reuse the model-build workspace; the ADMM metric must not alter fallback. */
    if (!Tinympc14_Admm_Reference_Model(&tinympc14_work)) return -1;
    Tinympc14_Online_Aux(&tinympc14_work);
#endif
    if (!Tinympc14_Admm_Model(&tinympc14_work)) return -1;

    back = 1 - tinympc14_pub_idx;
    memcpy(&tinympc14_pub[back], &tinympc14_work, sizeof(tinympc14_work));
    memcpy(tinympc14_pub_pinv[back], tinympc14_bd_pinv, sizeof(tinympc14_pub_pinv[back]));
#if TINYMPC14_CLOSED_CHAIN_AXIAL_INPUT
    /* 冻结工作点的逆虚功映射。等量反向轴力产生指定横滚矩，升降广义力为零。 */
    const float cos_psi = cosf(psi);
    const float tan_roll = (lL - lR) * cos_psi / (2.0f * TINYMPC14_HALF_TRACK);
    tinympc14_pub_roll_scale[back] = cos_psi / (1.0f + tan_roll * tan_roll);
#endif
    Tinympc14_Port_Enter_Critical();
    tinympc14_pub_idx = back;
    /* A control beat reading the new gains must also see the old sequence invalidated. */
    Tinympc14_Async_Drop();
    Tinympc14_Port_Exit_Critical();

    return 0;
}

static void Tinympc14_Uss(const tinympc14_model_t *m, const double *pinv,
                             const float *xref, const float *affine, float *fx, float *u_ss)
{
    int i,j;
    double rhs[TINYMPC14_NX];
    (void)fx;
    for(i=0;i<TINYMPC14_NX;i++)
    {
        double a=(double)xref[i]-affine[i];
        for(j=0;j<TINYMPC14_NX;j++) a-=(double)m->Ad[i*TINYMPC14_NX+j]*xref[j];
        rhs[i]=a;
    }
    for(i=0;i<TINYMPC14_NU;i++)
    {
        double a=0;
        for(j=0;j<TINYMPC14_NX;j++) a+=pinv[i*TINYMPC14_NX+j]*rhs[j];
        u_ss[i]=(float)a;
    }
}

void Tinympc14_Affine(const float xref[TINYMPC14_NX],
                         const float fed[TINYMPC14_NX],
                         const float ffwd[TINYMPC14_NU],
                         float u_ss[TINYMPC14_NU],
                         float u_ff[TINYMPC14_NU])
{
    int i;

    for (i = 0; i < TINYMPC14_NX; i++)
    {
        tinympc14_aff[i] = tinympc14_work.fd[i] + fed[i];
    }

    Tinympc14_Uss(&tinympc14_work, tinympc14_bd_pinv, xref, tinympc14_aff, tinympc14_fx, u_ss);

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        u_ff[i] = u_ss[i] + ffwd[i];
    }
}

static int Tinympc14_Pub_Snap(void)
{
    int idx;

    Tinympc14_Port_Enter_Critical();
    idx = tinympc14_pub_idx;
    Tinympc14_Port_Exit_Critical();

    return idx;
}

void Tinympc14_Affine_Pub(const float xref[TINYMPC14_NX],
                             const float fed[TINYMPC14_NX],
                             const float ffwd[TINYMPC14_NU],
                             float u_ss[TINYMPC14_NU],
                             float u_ff[TINYMPC14_NU])
{
    int i;
    int s = Tinympc14_Pub_Snap();
    const tinympc14_model_t *m = &tinympc14_pub[s];

    for (i = 0; i < TINYMPC14_NX; i++)
    {
        tinympc14_aff_pub[i] = m->fd[i] + fed[i];
    }

    Tinympc14_Uss(m, tinympc14_pub_pinv[s], xref, tinympc14_aff_pub, tinympc14_fx_pub, u_ss);

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        u_ff[i] = u_ss[i] + ffwd[i];
    }
}

void Tinympc14_Seed(float u_prev[TINYMPC14_NU])
{
    int i;

    {
        float zero[TINYMPC14_NX]={0};
        Tinympc14_Uss(&tinympc14_work,tinympc14_bd_pinv,zero,tinympc14_work.fd,tinympc14_fx,u_prev);
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        u_prev[i] = Tinympc14_Clip(u_prev[i], tinympc14_u_min[i], tinympc14_u_max[i]);
    }
}

void Tinympc14_Init(void)
{
    int i;

    tinympc14_status.iters = 0;
    tinympc14_status.converged = 0;
    tinympc14_status.pri = INFINITY;
    tinympc14_status.dua = INFINITY;

    tinympc14_counters = (tinympc14_counters_t){0};

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        tinympc14_u[i] = 0.0f;
    }

    tinympc14_last_raw[0] = INFINITY;
    tinympc14_last_raw[1] = INFINITY;
    tinympc14_last_raw_psi = INFINITY;
    tinympc14_model_ok = (Tinympc14_Model_Force(TINYMPC14_SCHED_L, TINYMPC14_SCHED_L, 0.0f, TINYMPC14_SCHED_L) == 0);
    tinympc14_last_raw[0] = TINYMPC14_SCHED_L;
    tinympc14_last_raw[1] = TINYMPC14_SCHED_L;
    tinympc14_last_raw_psi = 0.0f;
    tinympc14_last_roll_height = TINYMPC14_SCHED_L;
    Tinympc14_Async_Reset();
    {
        float seed[TINYMPC14_NU];
        Tinympc14_Seed(seed);
        for(i=0;i<TINYMPC14_NU;i++) tinympc14_u[i]=seed[i];
    }
}

/* Raw leg lengths and co-sampled IMU roll set only the roll inertia. */
static float Tinympc14_Roll_Height(float left, float right, float roll)
{
#if defined(TINYMPC14_ROLL_H_MIN)
    return Tinympc14_Clip((0.5f * left + 0.5f * right) * cosf(roll),
                         TINYMPC14_ROLL_H_MIN, TINYMPC14_ROLL_H_MAX);
#else
    (void)left;
    (void)right;
    (void)roll;
    return TINYMPC14_SCHED_L;
#endif
}

/* Both model reuse and asynchronous sampling use the same scheduling cell. */
static int Tinympc14_Model_Matches(float clamped_left, float clamped_right, float clamped_psi, float roll_height)
{
    /* Publish the upper endpoint once even when the final increment is under 5 mm. */
    if (clamped_left == TINYMPC14_L_MAX && clamped_right == TINYMPC14_L_MAX &&
        (tinympc14_last_raw[0] != TINYMPC14_L_MAX || tinympc14_last_raw[1] != TINYMPC14_L_MAX))
    {
        return 0;
    }
#if defined(TINYMPC14_ROLL_H_MIN)
    if (roll_height == TINYMPC14_ROLL_H_MIN && roll_height < tinympc14_last_roll_height)
    {
        return 0;
    }
#endif
    return fabsf(roll_height - tinympc14_last_roll_height) <= TINYMPC14_L_STEP
        && fabsf(clamped_left - tinympc14_last_raw[0]) <= TINYMPC14_L_STEP
        && fabsf(clamped_right - tinympc14_last_raw[1]) <= TINYMPC14_L_STEP
        && fabsf(clamped_psi - tinympc14_last_raw_psi) <= TINYMPC14_PSI_STEP;
}

static int Tinympc14_Model_Update_Height(float lL, float lR, float psi, float roll_height)
{
    float cL = Tinympc14_Clamp_Len(lL);
    float cR = Tinympc14_Clamp_Len(lR);
    float cp = Tinympc14_Clamp_Psi(psi);
    float rL = cL;
    float rR = cR;
    float rp = cp;
    if(!isfinite(lL)||!isfinite(lR)||!isfinite(psi)||!isfinite(roll_height))
    {
        tinympc14_counters.nan++;
        return -1;
    }

    if (Tinympc14_Model_Matches(rL, rR, rp, roll_height))
    {
        return 0;
    }

    if (Tinympc14_Model_Force(cL, cR, cp, roll_height) != 0)
    {
        tinympc14_model_ok=0;
        tinympc14_last_raw[0] = INFINITY;
        tinympc14_last_raw[1] = INFINITY;
        tinympc14_last_raw_psi = INFINITY;
        return -1;
    }

    tinympc14_last_raw[0] = rL;
    tinympc14_last_raw[1] = rR;
    tinympc14_last_raw_psi = rp;
    tinympc14_last_roll_height = roll_height;
    tinympc14_counters.lpv_updates++;
    tinympc14_model_ok=1;

    return 1;
}

int Tinympc14_Model_Update_Roll(float lL, float lR, float psi, float roll)
{
    if (!isfinite(roll)) return -1;
    return Tinympc14_Model_Update_Height(lL, lR, psi, Tinympc14_Roll_Height(lL, lR, roll));
}

int Tinympc14_Model_Update(float lL, float lR, float psi)
{
    /* Existing three-input callers use their original geometric scheduling point. */
    float left = Tinympc14_Clamp_Len(lL);
    float right = Tinympc14_Clamp_Len(lR);
    float vertical = cosf(Tinympc14_Clamp_Psi(psi));
    float tangent = (left - right) * vertical / (2.0f * TINYMPC14_HALF_TRACK);
    float height = 0.5f * (left + right) * vertical / sqrtf(1.0f + tangent * tangent);
    return Tinympc14_Model_Update_Height(lL, lR, psi, height);
}

/* Shift only actuator constraints; external compensation is not model feedforward. */
static int Tinympc14_Solve_Offset(const float x0[TINYMPC14_NX],
                       const float xref[TINYMPC14_NX],
                       const float fed[TINYMPC14_NX],
                       const float ffwd[TINYMPC14_NU],
                       const float u_prev[TINYMPC14_NU],
                       const float offset[TINYMPC14_NU], const float force_error[2],
                       float u_tot[TINYMPC14_NU], float u_ff[TINYMPC14_NU])
{
    float u_ss[TINYMPC14_NU];
    float f_eff[TINYMPC14_NX];
    float first_f[TINYMPC14_NX];
    float u_seed[TINYMPC14_NU];
    float upc[TINYMPC14_NU];
    float umin_s[TINYMPC14_NU];
    float umax_s[TINYMPC14_NU];
    float u_bare[TINYMPC14_NU];
    int i;

    if (!tinympc14_model_ok
        || !Tinympc14_All_Finite(x0, TINYMPC14_NX)
        || !Tinympc14_All_Finite(xref, TINYMPC14_NX)
        || !Tinympc14_All_Finite(fed, TINYMPC14_NX)
        || !Tinympc14_All_Finite(ffwd, TINYMPC14_NU))
    {
        tinympc14_counters.nan++;
        tinympc14_status.iters = 0;
        tinympc14_status.converged = 0;
        tinympc14_status.pri = INFINITY;
        tinympc14_status.dua = INFINITY;

        for (i = 0; i < TINYMPC14_NU; i++)
        {
            float p = isfinite(u_prev[i]) ? u_prev[i] : 0.0f;

            u_tot[i] = Tinympc14_Clip(p, tinympc14_u_min[i], tinympc14_u_max[i]) - offset[i];
        }

        return 0;
    }

    Tinympc14_Affine(xref, fed, ffwd, u_ss, u_ff);

    Tinympc14_Mul(tinympc14_fx, tinympc14_work.Bd, u_ff, TINYMPC14_NX, TINYMPC14_NU);

    for (i = 0; i < TINYMPC14_NX; i++)
    {
        f_eff[i] = tinympc14_work.fd[i] + fed[i] + tinympc14_fx[i];
        first_f[i] = f_eff[i] + tinympc14_work.Bd[i * NU + 4] * force_error[0]
                              + tinympc14_work.Bd[i * NU + 5] * force_error[1];
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        upc[i] = Tinympc14_Clip(isfinite(u_prev[i]) ? u_prev[i] : 0.0f,
                                   tinympc14_u_min[i], tinympc14_u_max[i]);
        umin_s[i] = tinympc14_u_min[i] - offset[i] - u_ff[i];
        umax_s[i] = tinympc14_u_max[i] - offset[i] - u_ff[i];
        u_seed[i] = Tinympc14_Clip(upc[i] - offset[i] - u_ff[i], umin_s[i], umax_s[i]);
    }

    if (!Tinympc14_Admm_Solve(&tinympc14_work,x0,xref,f_eff,first_f,u_seed,umin_s,umax_s,u_bare))
    {
        int j;
        for (i=0;i<TINYMPC14_NU;i++)
        {
            double acc=0;
            for(j=0;j<TINYMPC14_NX;j++)
                acc+=(double)tinympc14_work.Kinf[i*TINYMPC14_NX+j]*(x0[j]-xref[j]);
            u_bare[i]=(float)-acc;
        }
    }

    for (i = 0; i < TINYMPC14_NU; i++)
    {
        float lo = fmaxf(tinympc14_u_min[i], upc[i] - tinympc14_du_rate[i]);
        float hi = fminf(tinympc14_u_max[i], upc[i] + tinympc14_du_rate[i]);

        u_tot[i] = Tinympc14_Clip(u_bare[i] + u_ff[i] + offset[i], lo, hi) - offset[i];
    }

    return tinympc14_status.converged;
}

/* Standalone solver has no external axial compensation. */
int Tinympc14_Solve(const float x0[TINYMPC14_NX], const float xref[TINYMPC14_NX],
                   const float fed[TINYMPC14_NX], const float ffwd[TINYMPC14_NU],
                   const float u_prev[TINYMPC14_NU], float u_tot[TINYMPC14_NU])
{
    const float offset[TINYMPC14_NU] = {0.0f};
    float u_ff[TINYMPC14_NU];

    return Tinympc14_Solve_Offset(x0, xref, fed, ffwd, u_prev, offset, offset + 4, u_tot, u_ff);
}

const tinympc14_model_t *Tinympc14_Model(void)
{
    return &tinympc14_pub[Tinympc14_Pub_Snap()];
}

const float *Tinympc14_Sequence(void)
{
    return Tinympc14_Admm_Sequence();
}

/* ---- admm ---- */

/* Finite-horizon ADMM. Q/R are physical cost weights; rho is only a penalty.
 * Double workspace avoids cancellation between P*xref and the affine term.
 * Rebuild Riccati factors only when the LPV model changes. No heap use. */


/* Riccati P is symmetric. Store its lower triangle without changing precision. */
static double rk[NN][NU*NX], rq[NN][NU*NU], ra[NN][NX*NX], rp[NN][NX*(NX+1)/2];
static double adv[NH][NX], g[NH][NX], z[NN][NU], y[NN][NU];
static double xs[NH][NX], us[NN][NU], ps[NH][NX], ds[NN][NU];
static double ap[NN][NX], bp[NN][NU];
/* 模型更新时一次转换，迭代内复用相同的 double 值。 */
static double ad[NX * NX], bd[NX * NU];
static float sequence[NN*NU];
/* Model-build scratch is static so the embedded solve task has a bounded stack. */
static double pmat[NX*NX], pnew[NX*NX], btp[NU*NX], haug[NU][2*NU];

void Tinympc14_Admm_Reset(void)
{
    memset(adv,0,sizeof(adv)); memset(g,0,sizeof(g));
    memset(z,0,sizeof(z)); memset(y,0,sizeof(y));
}

#if defined(TINYMPC14_ONLINE_IFACE_VER)
static void Tinympc14_Online_Aux(tinympc14_model_t *model)
{
    int i;

    /* The final Riccati stage uses Pinf directly. Reuse those factors
     * for the public model and one-step feedback. */
    for (i = 0; i < NU * NX; i++) model->Kinf[i] = (float)rk[NN-1][i];
    for (i = 0; i < NU * NU; i++) model->QuuInv[i] = (float)rq[NN-1][i];
    for (i = 0; i < NX * NX; i++) model->AmBKt[i] = (float)ra[NN-1][i];
}
#endif

#if defined(TINYMPC14_ONLINE_IFACE_VER)
static int Tinympc14_Admm_Reference_Model(const tinympc14_model_t *m)
{
    int k,i,j,t,c;
    double bpa[NU];
    for(i=0;i<NX;i++) for(j=0;j<NX;j++)
        pmat[i*NX+j]=.5*((double)m->Pinf[i*NX+j]+m->Pinf[j*NX+i]);
    for(k=NN-1;k>=NN-1;k--)
    {
        for(i=0;i<NU;i++) for(j=0;j<NX;j++)
        {
            double a=0;
            for(t=0;t<NX;t++) a+=(double)m->Bd[t*NU+i]*pmat[t*NX+j];
            btp[i*NX+j]=a;
        }
        for(i=0;i<NU;i++) for(j=0;j<NU;j++)
        {
            double a=(i==j)?tinympc14_r_diag[i]+(double)TINYMPC14_RHO:0;
            for(t=0;t<NX;t++) a+=btp[i*NX+t]*m->Bd[t*NU+j];
            haug[i][j]=a; haug[i][NU+j]=(i==j)?1:0;
        }
        for(c=0;c<NU;c++)
        {
            double pivot=haug[c][c];
            if(!(pivot>0)||!isfinite(pivot)) return 0;
            for(j=0;j<2*NU;j++) haug[c][j]/=pivot;
            for(i=0;i<NU;i++) if(i!=c)
            {
                double a=haug[i][c];
                for(j=0;j<2*NU;j++) haug[i][j]-=a*haug[c][j];
            }
        }
        for(i=0;i<NU;i++) for(j=0;j<NU;j++) rq[k][i*NU+j]=haug[i][NU+j];
        /* Form B'PA once per column before applying the input Hessian inverse. */
        for (j = 0; j < NX; j++)
        {
            for (i = 0; i < NU; i++)
            {
                double a = 0.0;
                for (t = 0; t < NX; t++) a += btp[i*NX+t]*m->Ad[t*NX+j];
                bpa[i] = a;
            }
            for (i = 0; i < NU; i++)
            {
                double a = 0.0;
                for (t = 0; t < NU; t++) a += rq[k][i*NU+t]*bpa[t];
                rk[k][i*NX+j] = a;
            }
        }
        for(i=0;i<NX;i++) for(j=0;j<NX;j++)
        {
            double a=m->Ad[j*NX+i];
            for(t=0;t<NU;t++) a-=m->Bd[j*NU+t]*rk[k][t*NX+i];
            ra[k][i*NX+j]=a;
        }
    }
    return 1;
}

#endif

/* Scale the wheel-torque and axial-force consensus steps for active wheel
 * bounds and force slew limits. Hip torque and state metrics use rho. */
static double tinympc14_input_penalty[NU];

static double Tinympc14_Input_Penalty(int channel)
{
    return tinympc14_input_penalty[channel];
}

int Tinympc14_Admm_Model(const tinympc14_model_t *m)
{
    int k,i,j,t,c;
    double bpa[NU];
    double pacl[NX];

    for (i = 0; i < NX * NX; i++) ad[i] = m->Ad[i];
    for (i = 0; i < NX * NU; i++) bd[i] = m->Bd[i];
    for (i = 0; i < NU; i++)
    {
        tinympc14_input_penalty[i] = (double)TINYMPC14_RHO;
        if (i < 2 || i >= 4)
            tinympc14_input_penalty[i] = fmax(tinympc14_input_penalty[i], (double)tinympc14_r_diag[i]);
    }
    for(i=0;i<NX;i++) for(j=0;j<NX;j++)
        pmat[i*NX+j]=.5*((double)m->Pinf[i*NX+j]+m->Pinf[j*NX+i]);
    for(k=NN-1;k>=0;k--)
    {
        for (i = 0; i < NX; i++)
        {
            for (j = 0; j <= i; j++) rp[k][i*(i+1)/2+j] = pmat[i*NX+j];
        }
        /* 相邻列共用 B 的读取，每个内积仍按 t 递增累加。 */
        for (i = 0; i < NU; i++) for (j = 0; j < NX; j += 2)
        {
            double a0 = 0.0, a1 = 0.0;

            for (t = 0; t < NX; t++)
            {
                const double bt = bd[t * NU + i];
                a0 += bt * pmat[t * NX + j];
                a1 += bt * pmat[t * NX + j + 1];
            }
            btp[i * NX + j] = a0;
            btp[i * NX + j + 1] = a1;
        }
        for(i=0;i<NU;i++) for(j=0;j<NU;j++)
        {
            double a=(i==j)?tinympc14_r_diag[i]+Tinympc14_Input_Penalty(i):0;
            for(t=0;t<NX;t++) a+=btp[i*NX+t]*m->Bd[t*NU+j];
            haug[i][j]=a; haug[i][NU+j]=(i==j)?1:0;
        }
        for(c=0;c<NU;c++)
        {
            double pivot=haug[c][c];
            if(!(pivot>0)||!isfinite(pivot)) return 0;
            for(j=0;j<2*NU;j++) haug[c][j]/=pivot;
            for(i=0;i<NU;i++) if(i!=c)
            {
                double a=haug[i][c];
                for(j=0;j<2*NU;j++) haug[i][j]-=a*haug[c][j];
            }
        }
        for(i=0;i<NU;i++) for(j=0;j<NU;j++) rq[k][i*NU+j]=haug[i][NU+j];
        /* Form B'PA once per column before applying the input Hessian inverse. */
        for (j = 0; j < NX; j++)
        {
            for (i = 0; i < NU; i += 2)
            {
                double a0 = 0.0, a1 = 0.0;

                for (t = 0; t < NX; t++)
                {
                    const double at = ad[t * NX + j];
                    a0 += btp[i * NX + t] * at;
                    a1 += btp[(i + 1) * NX + t] * at;
                }
                bpa[i] = a0;
                bpa[i + 1] = a1;
            }
            for (i = 0; i < NU; i++)
            {
                double a = 0.0;
                for (t = 0; t < NU; t++) a += rq[k][i*NU+t]*bpa[t];
                rk[k][i*NX+j] = a;
            }
        }
        for (i = 0; i < NX; i += 2) for (j = 0; j < NX; j++)
        {
            double a0 = ad[j * NX + i], a1 = ad[j * NX + i + 1];

            for (t = 0; t < NU; t++)
            {
                const double bt = bd[j * NU + t];
                a0 -= bt * rk[k][t * NX + i];
                a1 -= bt * rk[k][t * NX + i + 1];
            }
            ra[k][i * NX + j] = a0;
            ra[k][(i + 1) * NX + j] = a1;
        }
        /* The preceding stage is the only reader of pnew. All factors for k=0
         * have been stored, so its unused cost-to-go need not be computed. */
        if (k == 0) break;
        /* Qbar + K' Rbar K + (A-BK)' P (A-BK), symmetric Joseph form.
         * Reuse P*(A-BK) column j for the whole lower triangle. */
        for (j = 0; j < NX; j++)
        {
            for (i = 0; i < NX; i += 2)
            {
                double a0 = 0.0, a1 = 0.0;

                for (t = 0; t < NX; t++)
                {
                    const double at = ra[k][j * NX + t];
                    a0 += pmat[i * NX + t] * at;
                    a1 += pmat[(i + 1) * NX + t] * at;
                }
                pacl[i] = a0;
                pacl[i + 1] = a1;
            }
            for (i = j; i < NX; i++)
            {
                double a = (i == j) ? tinympc14_q_diag[i]+(double)TINYMPC14_RHO : 0.0;
                for (t = 0; t < NU; t++)
                    a += rk[k][t*NX+i]*(tinympc14_r_diag[t]+Tinympc14_Input_Penalty(t))*rk[k][t*NX+j];
                for (t = 0; t < NX; t++) a += ra[k][i*NX+t]*pacl[t];
                pnew[i*NX+j] = a;
                pnew[j*NX+i] = a;
            }
        }
        memcpy(pmat, pnew, sizeof(pmat));
    }
    Tinympc14_Admm_Reset();
    return 1;
}

static double clip(double a,double l,double h) { return a<l?l:(a>h?h:a); }

/* Derivative of the minimum cumulative squared error on one interval:
 * slope * x + offset. Positive slopes and monotonicity include one-sided
 * derivatives at the ends of the feasible interval. */
typedef struct
{
    double end;
    double slope;
    double offset;
} tinympc14_projection_piece_t;

/* Restrict a translated derivative piece to the feasible input interval. */
static void Tinympc14_Projection_Append(tinympc14_projection_piece_t *pieces, int *count,
                                        double begin, double end, double low, double high,
                                        double slope, double offset)
{
    if (begin < low) begin = low;
    if (end > high) end = high;
    if (end > begin)
    {
        pieces[*count].end = end;
        pieces[*count].slope = slope;
        pieces[*count].offset = offset;
        ++*count;
    }
}

/* Euclidean projection onto all horizon box and slew limits. For a fixed
 * new input x, the best predecessor is the previous prefix minimizer clipped
 * to [x-rate,x+rate]. The prefix cost derivative therefore has two translated
 * tails and a flat middle, plus x-target[k]. Each node adds at most two
 * pieces. Backtracking the stored prefix minimizers recovers the joint optimum. */
static int project(int i, const float *lo, const float *hi, const float *prev)
{
    tinympc14_projection_piece_t buffers[2][2*NN];
    tinympc14_projection_piece_t *old = buffers[0], *next = buffers[1], *swap;
    double target[NN], minimum[NN], boxed[NN];
    const double rate = tinympc14_du_rate[i];
    double low = fmax(lo[i], prev[i] - rate);
    double high = fmin(hi[i], prev[i] + rate);
    int k, j, count = 1, feasible = 1;

    for (k = 0; k < NN; ++k)
    {
        target[k] = us[k][i] + y[k][i];
        boxed[k] = clip(target[k], k == 0 ? low : lo[i], k == 0 ? high : hi[i]);
        if (k > 0 && fabs(boxed[k] - boxed[k-1]) > rate) feasible = 0;
    }
    if (low > high) return 0;
    if (feasible)
    {
        for (k = 0; k < NN; ++k) z[k][i] = boxed[k];
        return 1;
    }

    minimum[0] = boxed[0];
    old[0].end = high;
    old[0].slope = 1.0;
    old[0].offset = -target[0];
    for (k = 1; k < NN; ++k)
    {
        const double next_low = fmax(lo[i], low - rate);
        const double next_high = fmin(hi[i], high + rate);
        const double center = minimum[k-1];
        double begin = low;
        int next_count = 0;

        for (j = 0; j < count && begin < center; ++j)
        {
            double end = fmin(old[j].end, center);
            Tinympc14_Projection_Append(next, &next_count, begin-rate, end-rate, next_low, next_high,
                                        old[j].slope+1.0, old[j].offset+old[j].slope*rate-target[k]);
            begin = old[j].end;
        }
        Tinympc14_Projection_Append(next, &next_count, center-rate, center+rate, next_low, next_high,
                                    1.0, -target[k]);
        begin = low;
        for (j = 0; j < count; ++j)
        {
            if (old[j].end > center)
            {
                double start = fmax(begin, center);
                Tinympc14_Projection_Append(next, &next_count, start+rate, old[j].end+rate, next_low, next_high,
                                            old[j].slope+1.0, old[j].offset-old[j].slope*rate-target[k]);
            }
            begin = old[j].end;
        }
        begin = next_low;
        minimum[k] = next_high;
        for (j = 0; j < next_count; ++j)
        {
            if (next[j].slope * next[j].end + next[j].offset >= 0.0)
            {
                minimum[k] = clip(-next[j].offset / next[j].slope, begin, next[j].end);
                break;
            }
            begin = next[j].end;
        }
        low = next_low;
        high = next_high;
        count = next_count;
        swap = old; old = next; next = swap;
    }
    z[NN-1][i] = minimum[NN-1];
    for (k = NN-2; k >= 0; --k)
        z[k][i] = clip(minimum[k], z[k+1][i] - rate, z[k+1][i] + rate);
    return 1;
}

/* Two successive ordinary ADMM sweeps supply a one-direction Anderson
 * prediction of the next input consensus and dual initial values. The scalar
 * coefficient minimizes the residual in the existing input penalty metric.
 * Every published solution still passes a complete, unmodified projection
 * and both physical residual tests. History is local to one solve. */
static void Tinympc14_Admm_Predict(double oldz[NN][NU], int first, int mix)
{
    static struct { double z, y, dz, dy; } history[NN][NU];
    double dot = 0.0, norm = 0.0, beta = 0.0;
    int k, i;

    for (k = 0; k < NN; ++k) for (i = 0; i < NU; ++i)
    {
        const double dz = z[k][i] - oldz[k][i], dy = us[k][i] - z[k][i];
        if (mix)
        {
            const double az = dz - history[k][i].dz, ay = dy - history[k][i].dy;
            const double weight = tinympc14_input_penalty[i];
            dot += weight * (dz * az + dy * ay);
            norm += weight * (az * az + ay * ay);
        }
        history[k][i].dz = dz;
        history[k][i].dy = dy;
    }
    if (mix && norm > 1e-20)
        beta = clip(-dot / norm, -0.8, 2.0);

    for (k = 0; k < NN; ++k) for (i = 0; i < NU; ++i)
    {
        const double zk = z[k][i], yk = y[k][i];
        if (!first && beta != 0.0)
        {
            z[k][i] += beta * (zk - history[k][i].z);
            y[k][i] += beta * (yk - history[k][i].y);
        }
        history[k][i].z = zk;
        history[k][i].y = yk;
    }
}

static int Tinympc14_Admm_Solve(const tinympc14_model_t *m,const float *x0,const float *ref,
                   const float *f,const float *first_f,const float *prev,const float *lo,const float *hi,float *u)
{
    double pri=INFINITY,dua=INFINITY;
    double previous_merit = INFINITY, terminal_ref[NX];
    int k,i,j,it,conv=0;
    const double rho=TINYMPC14_RHO;
    for (i = 0; i < NX; ++i)
    {
        double a = rho * ref[i];
        for (j = 0; j < NX; ++j) a -= m->Pinf[i*NX+j] * ref[j];
        terminal_ref[i] = a;
    }
    for(k=0;k<NN;k++)
    {
        double pf[NX];
        const float *step_f = k == 0 ? first_f : f;
        for (i = 0; i < NX; i++)
        {
            pf[i] = 0.0;
            for (j = 0; j <= i; j++) pf[i] += rp[k][i*(i+1)/2+j]*step_f[j];
            for (j = i+1; j < NX; j++) pf[i] += rp[k][j*(j+1)/2+i]*step_f[j];
        }
        for(i=0;i<NX;i++) { ap[k][i]=0; for(j=0;j<NX;j++) ap[k][i]+=ra[k][i*NX+j]*pf[j]; }
        for(i=0;i<NU;i++) { bp[k][i]=0; for(j=0;j<NX;j++) bp[k][i]+=bd[j*NU+i]*pf[j]; }
    }
    for(it=1;it<=TINYMPC14_MAX_ITER;it++)
    {
        double oldz[NN][NU];
#if !TINYMPC14_STATE_BOX_DISABLED
        double oldv[NH][NX];
        memcpy(oldv,adv,sizeof(adv));
#endif
        memcpy(oldz,z,sizeof(z));
        for(i=0;i<NX;i++)
        {
            ps[NN][i] = terminal_ref[i] - rho * (adv[NN][i] - g[NN][i]);
        }
        for(k=NN-1;k>=0;k--)
        {
            double r[NU],b[NU];
            // 相邻两行交错计算，共用向量读取；各行累加顺序保持不变。
            for (i = 0; i < NU; i += 2)
            {
                r[i] = -Tinympc14_Input_Penalty(i) * (z[k][i] - y[k][i]);
                r[i + 1] = -Tinympc14_Input_Penalty(i + 1) * (z[k][i + 1] - y[k][i + 1]);
                double b0 = r[i] + bp[k][i];
                double b1 = r[i + 1] + bp[k][i + 1];

                for (j = 0; j < NX; j++)
                {
                    const double next = ps[k + 1][j];
                    b0 += bd[j * NU + i] * next;
                    b1 += bd[j * NU + i + 1] * next;
                }
                b[i] = b0;
                b[i + 1] = b1;
            }
            for (i = 0; i < NU; i += 2)
            {
                double a0 = 0.0, a1 = 0.0;

                for (j = 0; j < NU; j++)
                {
                    const double bj = b[j];
                    a0 += rq[k][i * NU + j] * bj;
                    a1 += rq[k][(i + 1) * NU + j] * bj;
                }
                ds[k][i] = a0;
                ds[k][i + 1] = a1;
            }
            if (k == 0) break; /* forward control needs ds[0], never ps[0] */
            for (i = 0; i < NX; i += 2)
            {
                double a0 = -tinympc14_q_diag[i] * (double)ref[i] - rho * (adv[k][i] - g[k][i]) + ap[k][i];
                double a1 = -tinympc14_q_diag[i + 1] * (double)ref[i + 1] - rho * (adv[k][i + 1] - g[k][i + 1]) + ap[k][i + 1];

                for (j = 0; j < NX; j++)
                {
                    const double next = ps[k + 1][j];
                    a0 += ra[k][i * NX + j] * next;
                    a1 += ra[k][(i + 1) * NX + j] * next;
                }
                for (j = 0; j < NU; j++)
                {
                    const double rj = r[j];
                    a0 -= rk[k][j * NX + i] * rj;
                    a1 -= rk[k][j * NX + i + 1] * rj;
                }
                ps[k][i] = a0;
                ps[k][i + 1] = a1;
            }
        }
        for(i=0;i<NX;i++) xs[0][i]=x0[i];
        for(k=0;k<NN;k++)
        {
            const float *step_f = k == 0 ? first_f : f;
            for (i = 0; i < NU; i += 2)
            {
                double a0 = -ds[k][i], a1 = -ds[k][i + 1];

                for (j = 0; j < NX; j++)
                {
                    const double xj = xs[k][j];
                    a0 -= rk[k][i * NX + j] * xj;
                    a1 -= rk[k][(i + 1) * NX + j] * xj;
                }
                us[k][i] = a0;
                us[k][i + 1] = a1;
            }
            for (i = 0; i < NX; i += 2)
            {
                double a0 = step_f[i], a1 = step_f[i + 1];

                for (j = 0; j < NX; j++)
                {
                    const double xj = xs[k][j];
                    a0 += ad[i * NX + j] * xj;
                    a1 += ad[(i + 1) * NX + j] * xj;
                }
                for (j = 0; j < NU; j++)
                {
                    const double uj = us[k][j];
                    a0 += bd[i * NU + j] * uj;
                    a1 += bd[(i + 1) * NU + j] * uj;
                }
                xs[k + 1][i] = a0;
                xs[k + 1][i + 1] = a1;
            }
        }
        for(i=0;i<NU;i++) if(!project(i,lo,hi,prev)) break;
        if(i<NU) { pri=INFINITY; break; }
        pri=0; dua=0;
#if TINYMPC14_STATE_BOX_DISABLED
        memcpy(adv,xs,sizeof(adv));
#else
        for(k=0;k<NH;k++) for(i=0;i<NX;i++)
        {
            adv[k][i]=(k==0)?xs[k][i]:clip(xs[k][i]+g[k][i],tinympc14_x_min[i],tinympc14_x_max[i]);
            if(k>0) g[k][i]+=xs[k][i]-adv[k][i];
            const double primal = fabs(xs[k][i] - adv[k][i]);
            const double dual = rho * fabs(adv[k][i] - oldv[k][i]);
            if (primal > pri) pri = primal;
            if (dual > dua) dua = dual;
        }
#endif
        for(k=0;k<NN;k++) for(i=0;i<NU;i++)
        {
            const double difference = us[k][i] - z[k][i];
            const double primal = fabs(difference);
            const double dual = Tinympc14_Input_Penalty(i) * fabs(z[k][i] - oldz[k][i]);
            y[k][i] += difference;
            if (primal > pri) pri = primal;
            if (dual > dua) dua = dual;
            if(!isfinite(us[k][i])||!isfinite(z[k][i])) pri=INFINITY;
        }
        if(!isfinite(pri)||!isfinite(dua)) { Tinympc14_Admm_Reset(); break; }
        if(pri<TINYMPC14_PRI_TOL&&dua<TINYMPC14_DUA_TOL) { conv=1; break; }
        /* Limit extrapolation to the first 17 sweeps and suppress it when the
         * normalized residual grows. Difficult cases retain ordinary ADMM
         * for the remainder of the same iteration budget. */
        if (it < 18)
        {
            const double scaled_dual = dua * ((double)TINYMPC14_PRI_TOL / TINYMPC14_DUA_TOL);
            const double merit = pri > scaled_dual ? pri : scaled_dual;
            Tinympc14_Admm_Predict(oldz, it == 1, it > 1 && merit <= 1.5 * previous_merit);
            previous_merit = merit;
        }
    }
    tinympc14_status.iters=it>TINYMPC14_MAX_ITER?TINYMPC14_MAX_ITER:it;
    tinympc14_status.converged=conv; tinympc14_status.pri=(float)pri; tinympc14_status.dua=(float)dua;
    if(conv) for(k=0;k<NN;k++) for(i=0;i<NU;i++) sequence[k*NU+i]=(float)z[k][i];
    if(conv) for(i=0;i<NU;i++) u[i]=sequence[i];
    return conv;
}
const float *Tinympc14_Admm_Sequence(void) { return sequence; }

/* ---- async ---- */

typedef struct
{
    int      running;                          /* 1 = 允许求解 */
    int      seq_valid;                        /* 1 = 序列可消费 */
    int      buf;                              /* 当前序列缓冲 */
    int8_t   converged, publish_ms;            /* Last completion and latest publication outcome in this epoch. */
    uint8_t  has_published;                    /* base_tick identifies a committed sequence. */
    int      node_prev;                        /* 上一拍节点 */
    uint32_t base_tick;                        /* ms 提交时刻 */
    uint32_t epoch;                            /* explicit cancellation generation */
    uint32_t input_tick;                       /* snapshot age, milliseconds */
    int      has_input;                        /* 1 = 快照有效 */
    float    seq[2][TINYMPC14_NUU];
    float    seq_roll_rate[NH];                /* rad/s，与当前已提交命令同一解的 8 个状态节点 */
    float    u_prev[TINYMPC14_NU];          /* N 上拍总力 */
    float    sample_applied[TINYMPC14_NU];  /* 与 x0 配对的接纳命令，前台继续消费时保持 */
    float    x0[TINYMPC14_NX];
    float    xref[TINYMPC14_NX];
    float    fed[TINYMPC14_NX];
    float    ffwd[TINYMPC14_NU];
    float    force_comp[2];                    /* N output compensation, frozen with the control beat */
    float    force_error[2];                   /* N 当前估计的已建立轴力与接纳总力之差 */
    unsigned int force_valid;                  /* 本拍左/右反馈时标配对有效位 */
    float    pending_diff_impulse;             /* N·s，与 x0 同拍的去偏差力冲量，仅接地消费 */
    float    seq_force_error[2];               /* N 已提交预测在首节点使用的差值 */
    float    ground_load[3];                   /* N 两侧转弯净外载、按标称半轮距换算的倾覆差模力 */
    float    ground_comp[2];                   /* N 与同一物理快照绑定的轴力补偿 */
    int      has_ground_load;                  /* 当前快照接受按 LPV 工作点重投影 */
    float    roll_increment;                   /* rad/s 最近前台实际使用的扰动 */
    float    l[2];                             /* m 调度腿长 */
} tinympc14_async_t;

static tinympc14_async_t tinympc14_async;

static void Tinympc14_Clear_Solve_Diagnostic(void)
{
    tinympc14_async.converged = -1;
    tinympc14_async.publish_ms = TINYMPC14_PUB_NONE;
    tinympc14_async.has_published = 0;
}

void Tinympc14_Async_Reset(void)
{
    memset(&tinympc14_async, 0, sizeof(tinympc14_async));
    Tinympc14_Clear_Solve_Diagnostic();
    tinympc14_async.running = 1;
    tinympc14_async.node_prev = -1;
    Tinympc14_Seed(tinympc14_async.u_prev);
}

static float Tinympc14_Tip(float roll, float height)
{
    return TINYMPC14_ROLL_TIP_GAIN * TINYMPC14_M_SUP * TINYMPC14_G *
           (height + TINYMPC14_H_OFF) / TINYMPC14_ROLL_TRACK * roll;
}

/* Convert the measured moment and two axial loads with one model slot. */
/* 接地状态的 yaw 是相对积分目标的误差，每个预测节点扣除参考航向推进量。
 * 前台反馈和后台求解均在各自 LPV 投影后加入，避免被物理力重投影覆盖。 */
static void Tinympc14_Ground_Project(int slot, const float load[3], float yaw_rate, float out[NX])
{
    const tinympc14_model_t *model = &tinympc14_pub[slot];
    float tip = load[2];
#if TINYMPC14_CLOSED_CHAIN_AXIAL_INPUT
    tip *= tinympc14_pub_roll_scale[slot];
#endif
    const float left = load[0] + tip, right = load[1] - tip;
    for (int i = 0; i < NX; i++)
        out[i] = model->Bd[i * NU + 4] * left + model->Bd[i * NU + 5] * right;
    out[2] -= TINYMPC14_TS * yaw_rate;
}

/* Opposite equivalent forces affect prediction only, never motor feedforward. */
void Tinympc14_Roll_Disturbance(float roll, float height, float fed[NX])
{
    const float load[3] = {0.0f, 0.0f, Tinympc14_Tip(roll, height)};
    float projected[NX];
    if (!isfinite(load[2])) return;
    Tinympc14_Port_Enter_Critical();
    Tinympc14_Ground_Project(tinympc14_pub_idx, load, 0.0f, projected);
    Tinympc14_Port_Exit_Critical();
    for (int i = 0; i < NX; i++) fed[i] += projected[i];
}

void Tinympc14_Async_Run(void)
{
    Tinympc14_Port_Enter_Critical();
    tinympc14_async.running = 1;
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Applied(const float u_applied[TINYMPC14_NU])
{
    int i;
    if(!Tinympc14_All_Finite(u_applied,TINYMPC14_NU))
    {
        tinympc14_counters.nan++;
        return;
    }
    Tinympc14_Port_Enter_Critical();
    for(i=0;i<TINYMPC14_NU;i++)
        tinympc14_async.u_prev[i]=Tinympc14_Clip(u_applied[i],tinympc14_u_min[i],tinympc14_u_max[i]);
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Drop(void)
{
    Tinympc14_Port_Enter_Critical();
    tinympc14_async.seq_valid = 0;
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Invalidate(void)
{
    Tinympc14_Port_Enter_Critical();
    tinympc14_async.seq_valid = 0;
    tinympc14_async.running = 0;
    tinympc14_async.epoch++;
    Tinympc14_Clear_Solve_Diagnostic();
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Input(const float x0[TINYMPC14_NX],
                              const float xref[TINYMPC14_NX],
                              const float fed[TINYMPC14_NX],
                              const float ffwd[TINYMPC14_NU],
                              float lL, float lR)
{
    Tinympc14_Async_Input_At(x0, xref, fed, ffwd, lL, lR, Tinympc14_Port_Tick());
}

/* State age and prediction nodes are anchored to measurement time, including dispatch delay. */
void Tinympc14_Async_Input_At(const float x0[TINYMPC14_NX],
                            const float xref[TINYMPC14_NX],
                            const float fed[TINYMPC14_NX],
                            const float ffwd[TINYMPC14_NU],
                            float lL, float lR, uint32_t sample_tick)
{
    const float zero[2] = {0.0f, 0.0f};
    Tinympc14_Async_Input_Force_At(x0, xref, fed, ffwd, lL, lR, sample_tick, zero, 0u);
}

void Tinympc14_Async_Input_Force_At(const float x0[TINYMPC14_NX], const float xref[TINYMPC14_NX],
                                    const float fed[TINYMPC14_NX], const float ffwd[TINYMPC14_NU],
                                    float lL, float lR, uint32_t sample_tick, const float error[2], unsigned int valid)
{
    Tinympc14_Port_Enter_Critical();
    tinympc14_async.force_valid = 0u;
    tinympc14_async.pending_diff_impulse = 0.0f;
    tinympc14_async.has_ground_load = 0;
    memcpy(tinympc14_async.sample_applied, tinympc14_async.u_prev, sizeof(tinympc14_async.sample_applied));
    for (unsigned int side = 0u; side < 2u; side++)
    {
        tinympc14_async.force_error[side] = 0.0f;
        if ((valid & (1u << side)) && isfinite(error[side]))
        {
            const float span = tinympc14_u_max[4u + side] - tinympc14_u_min[4u + side];
            tinympc14_async.force_error[side] = Tinympc14_Clip(error[side], -span, span);
            tinympc14_async.force_valid |= 1u << side;
        }
    }
    memcpy(tinympc14_async.x0, x0, sizeof(tinympc14_async.x0));
    memcpy(tinympc14_async.xref, xref, sizeof(tinympc14_async.xref));
    memcpy(tinympc14_async.fed, fed, sizeof(tinympc14_async.fed));
    memcpy(tinympc14_async.ffwd, ffwd, sizeof(tinympc14_async.ffwd));
    tinympc14_async.l[0] = lL;
    tinympc14_async.l[1] = lR;
    tinympc14_async.has_input = 1;
    tinympc14_async.input_tick = sample_tick;
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Input_Ground_At(const float x0[NX], const float xref[NX], float lL, float lR,
                                     uint32_t tick, const tinympc14_ground_observation_t *observation)
{
    const float zero[NX] = {0.0f};
    Tinympc14_Port_Enter_Critical();
    Tinympc14_Async_Input_Force_At(x0, xref, zero, zero, lL, lR, tick, observation->error, observation->valid);
    if (tinympc14_async.force_valid == 3u && isfinite(observation->pending_diff_impulse))
        tinympc14_async.pending_diff_impulse = observation->pending_diff_impulse;
    for (int side = 0; side < 2; side++)
    {
        tinympc14_async.ground_comp[side] = observation->comp[side];
        tinympc14_async.ground_load[side] = observation->turn[side] + observation->comp[side];
    }
    tinympc14_async.ground_load[2] = Tinympc14_Tip(x0[10], observation->height);
    tinympc14_async.has_ground_load = Tinympc14_All_Finite(tinympc14_async.ground_load, 3);
    if (!tinympc14_async.has_ground_load) tinympc14_async.fed[0] = NAN;
    Tinympc14_Port_Exit_Critical();
}

float Tinympc14_Async_Roll_Increment(void)
{
    return tinympc14_async.roll_increment;
}

/* A failed snapshot invalidates the active sequence and leaves the solver running. */
static void Tinympc14_Async_Reject(volatile unsigned *reason, uint32_t age, uint32_t epoch, int8_t outcome)
{
    Tinympc14_Port_Enter_Critical();
    tinympc14_async.seq_valid = 0;
    if (tinympc14_async.epoch == epoch)
    {
        tinympc14_async.converged = outcome == TINYMPC14_PUB_UNCONVERGED ? 0 : -1;
        tinympc14_async.publish_ms = outcome;
    }
    ++*reason;
    tinympc14_counters.solve_age_ms = age;
    Tinympc14_Port_Exit_Critical();
}

typedef struct
{
    float x0[NX], xref[NX], fed[NX];
    float ffwd[NU], applied[NU], offset[NU];
    float force_error[2];
    float ground_load[3];
    int has_ground_load;
    float left, right;
    uint32_t tick;
} tinympc14_solve_input_t;

/* Caller holds the exchange lock. Capture the whole control beat, including
 * the applied force anchor and axial compensation used for its constraints. */
static void Tinympc14_Async_Copy_Input(tinympc14_solve_input_t *sample)
{
    memcpy(sample->x0, tinympc14_async.x0, sizeof(sample->x0));
    memcpy(sample->xref, tinympc14_async.xref, sizeof(sample->xref));
    memcpy(sample->fed, tinympc14_async.fed, sizeof(sample->fed));
    memcpy(sample->ffwd, tinympc14_async.ffwd, sizeof(sample->ffwd));
    memcpy(sample->applied, tinympc14_async.sample_applied, sizeof(sample->applied));
    const float *offset = tinympc14_async.has_ground_load ? tinympc14_async.ground_comp : tinympc14_async.force_comp;
    sample->offset[4] = offset[0];
    sample->offset[5] = offset[1];
    memcpy(sample->force_error, tinympc14_async.force_error, sizeof(sample->force_error));
    memcpy(sample->ground_load, tinympc14_async.ground_load, sizeof(sample->ground_load));
    sample->has_ground_load = tinympc14_async.has_ground_load;
    sample->left = tinympc14_async.l[0];
    sample->right = tinympc14_async.l[1];
    sample->tick = tinympc14_async.input_tick;
}

void Tinympc14_Async_Solve(void)
{
    Tinympc14_Async_Solve_Observed(NULL);
}

void Tinympc14_Async_Solve_Observed(tinympc14_publish_hook_t on_publish)
{
    tinympc14_solve_input_t sample = {0};
    float u_tot[TINYMPC14_NU];
    float u_ff[TINYMPC14_NU];
    uint32_t base, age;
    uint32_t epoch;
    int run, updated;
    int conv;

    Tinympc14_Port_Enter_Critical();
    run = tinympc14_async.running && tinympc14_async.has_input;
    epoch = tinympc14_async.epoch;
    if (run)
    {
        tinympc14_counters.solve_attempt++;
        tinympc14_async.publish_ms = TINYMPC14_PUB_PENDING;
        Tinympc14_Async_Copy_Input(&sample);
    }
    Tinympc14_Port_Exit_Critical();

    if (!run)
    {
        return;
    }

    base = sample.tick;
    age = Tinympc14_Port_Tick() - base;
    if (age >= TINYMPC14_NODE_TICKS * TINYMPC14_NN)
    {
        Tinympc14_Async_Reject(&tinympc14_counters.reject_start_age, age, epoch, TINYMPC14_PUB_START_LATE);
        return;
    }
    if (!isfinite(sample.left) || !isfinite(sample.right) || !Tinympc14_All_Finite(sample.x0, NX)
        || !Tinympc14_All_Finite(sample.xref, NX) || !Tinympc14_All_Finite(sample.fed, NX)
        || !Tinympc14_All_Finite(sample.ffwd, NU))
    {
        Tinympc14_Async_Reject(&tinympc14_counters.reject_input, Tinympc14_Port_Tick() - base, epoch, TINYMPC14_PUB_BAD_INPUT);
        return;
    }
    updated = Tinympc14_Model_Update_Roll(sample.left, sample.right, 0.5f * (sample.x0[4] + sample.x0[6]), sample.x0[10]);
    if (updated < 0)
    {
        Tinympc14_Async_Reject(&tinympc14_counters.reject_input, Tinympc14_Port_Tick() - base, epoch, TINYMPC14_PUB_BAD_INPUT);
        return;
    }
    if (updated)
    {
        /* Building the LPV factors can span several estimator beats. Solve
         * from the newest complete sample only if those factors apply to it.
         * Both the prediction and expiry then use that measurement's tick. */
        Tinympc14_Port_Enter_Critical();
        if (tinympc14_async.running && tinympc14_async.has_input && tinympc14_async.epoch == epoch
            && tinympc14_async.input_tick != base)
        {
            uint32_t now = Tinympc14_Port_Tick();
            float left = tinympc14_async.l[0], right = tinympc14_async.l[1];
            float psi = 0.5f * (tinympc14_async.x0[4] + tinympc14_async.x0[6]);
            if (now - tinympc14_async.input_tick < now - base
                && isfinite(left) && isfinite(right) && isfinite(psi)
                && Tinympc14_Model_Matches(Tinympc14_Clamp_Len(left), Tinympc14_Clamp_Len(right),
                                           Tinympc14_Clamp_Psi(psi),
                                           Tinympc14_Roll_Height(left, right, tinympc14_async.x0[10])))
            {
                Tinympc14_Async_Copy_Input(&sample);
                base = sample.tick;
            }
        }
        Tinympc14_Port_Exit_Critical();
    }
    /* Only this task publishes model slots. Project after any LPV refresh and snapshot recapture. */
    if (sample.has_ground_load)
        Tinympc14_Ground_Project(tinympc14_pub_idx, sample.ground_load, sample.xref[3], sample.fed);
    conv = Tinympc14_Solve_Offset(sample.x0, sample.xref, sample.fed, sample.ffwd,
                                  sample.applied, sample.offset, sample.force_error, u_tot, u_ff);

    if (!conv)
    {
        /* An unusable initial model can fail the solver guard before iteration one. */
        volatile unsigned *reason = tinympc14_status.iters == 0 ? &tinympc14_counters.reject_input
                                                               : &tinympc14_counters.reject_convergence;
        Tinympc14_Async_Reject(reason, Tinympc14_Port_Tick() - base, epoch,
                               tinympc14_status.iters == 0 ? TINYMPC14_PUB_BAD_INPUT : TINYMPC14_PUB_UNCONVERGED);
        return;
    }

    Tinympc14_Port_Enter_Critical();

    const uint32_t publish_tick = Tinympc14_Port_Tick();
    age = publish_tick - base;
    tinympc14_counters.solve_age_ms = age;
    if (!tinympc14_async.running || tinympc14_async.epoch != epoch)
    {
        tinympc14_counters.reject_epoch++;
        Tinympc14_Port_Exit_Critical();
        return;
    }
    if (age >= TINYMPC14_NODE_TICKS * TINYMPC14_NN)
    {
        tinympc14_counters.reject_finish_age++;
        tinympc14_async.converged = 1;
        tinympc14_async.publish_ms = TINYMPC14_PUB_FINISH_LATE;
        Tinympc14_Port_Exit_Critical();
        return;
    }

    {
        int back = 1 - tinympc14_async.buf;

        memcpy(tinympc14_async.seq[back], Tinympc14_Sequence(), sizeof(tinympc14_async.seq[back]));
        {
            int k,i;
            for(k=0;k<TINYMPC14_NN;k++) for(i=0;i<TINYMPC14_NU;i++)
                tinympc14_async.seq[back][k*TINYMPC14_NU+i]+=u_ff[i];
        }
        tinympc14_async.buf = back;
        for (int k = 0; k < NH; k++) tinympc14_async.seq_roll_rate[k] = (float)xs[k][11];
        memcpy(tinympc14_async.seq_force_error, sample.force_error, sizeof(sample.force_error));
        tinympc14_async.base_tick = base;
        tinympc14_async.node_prev = -1;
        tinympc14_async.seq_valid = 1;
    }

    tinympc14_counters.commit++;
    if (on_publish) on_publish(publish_tick, age, tinympc14_async.has_published, updated);
    tinympc14_async.has_published = 1;
    tinympc14_async.converged = 1;
    tinympc14_async.publish_ms = (int8_t)age;
    Tinympc14_Port_Exit_Critical();
}

void Tinympc14_Async_Step(void)
{
    Tinympc14_Async_Step_Period(TINYMPC14_NODE_TICKS);
}

static float Tinympc14_Rate_Scale(uint32_t control_ms)
{
    return fminf((float)control_ms / (float)TINYMPC14_NODE_TICKS, 1.0f);
}

void Tinympc14_Async_Step_Period(uint32_t control_ms)
{
    const float force_comp[2] = {0.0f, 0.0f};

    Tinympc14_Async_Step_Compensated(control_ms, force_comp);
}

static float Tinympc14_Async_Consume(uint32_t control_ms, const float force_comp[2], float lower_share, int grounded,
                                    tinympc14_beat_diag_t *diag);

void Tinympc14_Async_Step_Compensated(uint32_t control_ms, const float force_comp[2])
{
    (void)Tinympc14_Async_Consume(control_ms, force_comp, 0.0f, 0, NULL);
}

float Tinympc14_Async_Step_Lowering(uint32_t control_ms, const float force_comp[2], float lower_share)
{
    return Tinympc14_Async_Consume(control_ms, force_comp, lower_share, 1, NULL);
}

float Tinympc14_Async_Step_Lowering_Diag(uint32_t control_ms, const float force_comp[2], float lower_share,
                                         tinympc14_beat_diag_t *diag)
{
    return Tinympc14_Async_Consume(control_ms, force_comp, lower_share, 1, diag);
}

/* Limit physical total force once; return the MPC component for Output_Apply. */
static float Tinympc14_Async_Consume(uint32_t control_ms, const float force_comp[2], float lower_share, int grounded,
                                    tinympc14_beat_diag_t *diag)
{
    float x0[TINYMPC14_NX];
    float xref[TINYMPC14_NX];
    float fed[TINYMPC14_NX];
    float ffwd[TINYMPC14_NU];
    float u_prev[TINYMPC14_NU];
    float u_ss[TINYMPC14_NU];
    float u_ff[TINYMPC14_NU];
    float u_bare[TINYMPC14_NU];
    float offset[TINYMPC14_NU] = {0.0f};
    float common_cap = INFINITY;
    float holding_diff = 0.0f;
    float sensed_delta[NU] = {0.0f};
    float axial_lo[2], axial_hi[2];
    float lL, lR;
    uint32_t tick;
    uint32_t input_tick;
    int has;
    int run;
    int idx = -1;
    int i, j;
    const float rate_scale = Tinympc14_Rate_Scale(control_ms);

    if (diag) *diag = (tinympc14_beat_diag_t){2u, -1, -1, TINYMPC14_PUB_NONE};
    offset[4] = isfinite(force_comp[0]) ? force_comp[0] : 0.0f;
    offset[5] = isfinite(force_comp[1]) ? force_comp[1] : 0.0f;

    Tinympc14_Port_Enter_Critical();
    tinympc14_async.force_comp[0] = offset[4];
    tinympc14_async.force_comp[1] = offset[5];
    run = tinympc14_async.running;
    has = tinympc14_async.has_input;
    tinympc14_async.roll_increment = 0.0f;
    input_tick = tinympc14_async.input_tick;
    memcpy(x0, tinympc14_async.x0, sizeof(x0));
    memcpy(xref, tinympc14_async.xref, sizeof(xref));
    memcpy(fed, tinympc14_async.fed, sizeof(fed));
    memcpy(ffwd, tinympc14_async.ffwd, sizeof(ffwd));
    memcpy(u_prev, tinympc14_async.u_prev, sizeof(u_prev));
    lL = tinympc14_async.l[0];
    lR = tinympc14_async.l[1];
    Tinympc14_Port_Exit_Critical();

    tick = Tinympc14_Port_Tick();

    if (!run)
    {
        for (i = 0; i < TINYMPC14_NU; i++)
        {
            float p = isfinite(u_prev[i]) ? u_prev[i] : 0.0f;

            tinympc14_u[i] = Tinympc14_Clip(p, tinympc14_u_min[i], tinympc14_u_max[i]) - offset[i];
        }

        return 0.0f;
    }

    if (!has || !isfinite(lL) || !isfinite(lR)
        || !Tinympc14_All_Finite(x0, TINYMPC14_NX)
        || !Tinympc14_All_Finite(xref, TINYMPC14_NX)
        || !Tinympc14_All_Finite(fed, TINYMPC14_NX)
        || !Tinympc14_All_Finite(ffwd, TINYMPC14_NU))
    {
        tinympc14_counters.nan++;
        tinympc14_counters.fallback++;
        Tinympc14_Async_Drop();
        Tinympc14_Port_Enter_Critical();
        tinympc14_async.epoch++;
        Tinympc14_Clear_Solve_Diagnostic();
        Tinympc14_Port_Exit_Critical();

        for (i = 0; i < TINYMPC14_NU; i++)
        {
            float p = isfinite(u_prev[i]) ? u_prev[i] : 0.0f;
            float u = Tinympc14_Clip(p, tinympc14_u_min[i], tinympc14_u_max[i]);

            tinympc14_async.u_prev[i] = u;
            tinympc14_u[i] = u - offset[i];
        }

        return 0.0f;
    }

    Tinympc14_Port_Enter_Critical();
    if (tinympc14_async.has_ground_load)
        Tinympc14_Ground_Project(tinympc14_pub_idx, tinympc14_async.ground_load, xref[3], fed);
    tinympc14_async.roll_increment = fed[11];
    Tinympc14_Affine_Pub(xref, fed, ffwd, u_ss, u_ff);

    if (diag)
    {
        diag->converged = tinympc14_async.converged;
        diag->publish_ms = tinympc14_async.publish_ms;
        diag->expired = tinympc14_async.has_published ?
            (int8_t)((tick - tinympc14_async.base_tick) >= TINYMPC14_NODE_TICKS * TINYMPC14_NN) : -1;
    }
    if (tinympc14_async.seq_valid)
    {
        uint32_t d = tick - tinympc14_async.base_tick;
        uint32_t node = d / TINYMPC14_NODE_TICKS;

        if (node < (uint32_t)TINYMPC14_NN)
        {
            idx = (int)node;

            if (idx <= tinympc14_async.node_prev)
            {
                tinympc14_counters.node_repeat++;
            }

            tinympc14_async.node_prev = idx;
            memcpy(u_bare, tinympc14_async.seq[tinympc14_async.buf] + idx * TINYMPC14_NU,
                   TINYMPC14_NU * sizeof(float));
        }
        else
        {
            tinympc14_async.seq_valid = 0;
            tinympc14_counters.stale++;
        }
    }

    if ((uint32_t)(tick-input_tick)>=TINYMPC14_NODE_TICKS*TINYMPC14_NN)
    {
        /* No fresh observation: do not keep solving the same sampled state. */
        if(idx<0) tinympc14_counters.fallback++;
        for(i=0;i<TINYMPC14_NU;i++) tinympc14_u[i]=u_prev[i]-offset[i];
        Tinympc14_Port_Exit_Critical();
        return 0.0f;
    }

    if (diag) diag->source = idx < 0 ? 1u : 0u;

    /* A measured force error acts for one node. The terminal first-action response is
     * G = H^-1 B' P B_ax = I(:,ax) - H^-1(:,ax) (R_ax + rho I).
     * A committed node already priced its own forecast. Apply only newer information
     * against that node's expected error (zero beyond node zero). No change to the force anchor. */
    const tinympc14_model_t *model = Tinympc14_Model();
    const uint32_t feedback_age = tick - input_tick;
    if (feedback_age < TINYMPC14_NODE_TICKS)
    {
        for (unsigned int side = 0u; side < 2u; side++)
        {
            if (!(tinympc14_async.force_valid & (1u << side))) continue;
            const unsigned int ch = 4u + side;
            const float error = tinympc14_async.force_error[side];
            const float expected = idx == 0 ? tinympc14_async.seq_force_error[side] : 0.0f;
            for (i = 0; i < NU; i++)
            {
                const float gain = ((unsigned int)i == ch ? 1.0f : 0.0f)
                                 - model->QuuInv[i * NU + ch] * (tinympc14_r_diag[ch] + TINYMPC14_RHO);
                sensed_delta[i] += gain * error;
                if (idx >= 0 && (int32_t)(input_tick - tinympc14_async.base_tick) >= idx * (int)TINYMPC14_NODE_TICKS)
                    u_bare[i] -= gain * (error - expected);
            }
        }
    }

    if (idx>=0)
    {
        /* Committed sequence already contains the snapshot's u_ff. */
        for(i=0;i<TINYMPC14_NU;i++) u_bare[i]-=u_ff[i];
    }

    if (idx < 0)
    {
        const tinympc14_model_t *m = Tinympc14_Model();

        for (i = 0; i < TINYMPC14_NU; i++)
        {
            float acc = 0.0f;

            for (j = 0; j < TINYMPC14_NX; j++)
            {
                acc += m->Kinf[i * TINYMPC14_NX + j] * (x0[j] - xref[j]);
            }

            u_bare[i] = u_ss[i] + ffwd[i] - acc - u_ff[i] - sensed_delta[i];
        }

        tinympc14_counters.fallback++;
    }

    if (grounded)
    {
        /* Keep the steady differential for the current posture. Only the transient
         * differential in the last applied pair needs time to unwind. */
        float steady_diff = (u_ff[4] + offset[4]) - (u_ff[5] + offset[5]);
        for (j = 0; j < TINYMPC14_NX; j++)
        {
            if (j != 11)
                steady_diff -= (model->Kinf[4 * NX + j] - model->Kinf[5 * NX + j]) * (x0[j] - xref[j]);
        }
        steady_diff = Tinympc14_Clip(steady_diff, tinympc14_u_min[4] - tinympc14_u_max[5],
                                    tinympc14_u_max[4] - tinympc14_u_min[5]);
        const float excess = (u_prev[4] - u_prev[5]) - steady_diff;
        const float step = rate_scale * (tinympc14_du_rate[4] + tinympc14_du_rate[5]);
        /* The observer reports the untransmitted differential impulse after the
         * first-node innovation. Read the same stamped pair as x0 and force_error. */
        float impulse = feedback_age < TINYMPC14_NODE_TICKS ? tinympc14_async.pending_diff_impulse : 0.0f;
        if (step > 0.0f && fabsf(excess) > step)
        {
            /* Fastest permitted reversal starting at this beat: sum |excess|-k*step
             * for k >= 1 while positive. A leg at its box can only prolong it. */
            const float n = floorf(fabsf(excess) / step);
            impulse += copysignf(0.001f * (float)control_ms * n *
                                 (fabsf(excess) - 0.5f * (n + 1.0f) * step), excess);
        }
        const float roll_gain = (model->Bd[11 * NU + 4] - model->Bd[11 * NU + 5]) / (2.0f * TINYMPC14_TS);
        float rate_error = roll_gain * impulse;
        /* An active sequence is open loop between solves. Close its differential
         * force on the measured rate error relative to that trajectory. At a half
         * node, compare rates at the observation timestamp, including any first-node
         * force already predicted. Terminal fallback already uses the current rate. */
        const uint32_t sample_age = input_tick - tinympc14_async.base_tick;
        if (idx >= 0 && feedback_age < TINYMPC14_NODE_TICKS && sample_age < NN * TINYMPC14_NODE_TICKS)
        {
            const uint32_t k = sample_age / TINYMPC14_NODE_TICKS;
            const float fraction = (float)(sample_age % TINYMPC14_NODE_TICKS) / (float)TINYMPC14_NODE_TICKS;
            const float start = tinympc14_async.seq_roll_rate[k];
            const float predicted = start + fraction * (tinympc14_async.seq_roll_rate[k + 1u] - start);
            rate_error += x0[11] - predicted;
        }
        const float rate_gain = model->Kinf[4 * NX + 11] - model->Kinf[5 * NX + 11];
        const float correction = 0.5f * rate_gain * rate_error;
        u_bare[4] -= correction;
        u_bare[5] += correction;
    }
    if (lower_share > 0.0f)
    {
        const tinympc14_model_t *m = Tinympc14_Model();
        const float steady = u_ff[4] + u_ff[5] + offset[4] + offset[5];
        float common = steady - sensed_delta[4] - sensed_delta[5];
        holding_diff = (u_ff[4] + offset[4]) - (u_ff[5] + offset[5]) -
                       (m->Kinf[4 * TINYMPC14_NX + 10] - m->Kinf[5 * TINYMPC14_NX + 10]) * (x0[10] - xref[10]);

        /* Use the unbounded terminal common-mode demand to identify unloading,
         * including vertical braking. Keep the predictive node's differential. */
        for (j = 0; j < TINYMPC14_NX; j++)
            common -= (m->Kinf[4 * TINYMPC14_NX + j] + m->Kinf[5 * TINYMPC14_NX + j]) * (x0[j] - xref[j]);
        if (common < steady) common_cap = common;
    }
    Tinympc14_Port_Exit_Critical();

    const float diff = (u_bare[4] + u_ff[4] + offset[4]) - (u_bare[5] + u_ff[5] + offset[5]);
    for (i = 0; i < TINYMPC14_NU; i++)
    {
        float prev = Tinympc14_Clip(u_prev[i], tinympc14_u_min[i], tinympc14_u_max[i]);
        float rate = rate_scale * tinympc14_du_rate[i];
        float lo = fmaxf(tinympc14_u_min[i], prev - rate);
        float hi = fminf(tinympc14_u_max[i], prev + rate);

        u_bare[i] = Tinympc14_Clip(u_bare[i] + u_ff[i] + offset[i], lo, hi);

        if (i >= 4)
        {
            axial_lo[i - 4] = lo;
            axial_hi[i - 4] = hi;
        }
    }

    const float original = u_bare[4] + u_bare[5];
    if (common_cap < original)
    {
        float cap = fmaxf(common_cap, axial_lo[0] + axial_lo[1]);
        const float bounded_diff = u_bare[4] - u_bare[5];

        /* Keep the available static restoring differential if the normal action
         * already delivers it. This also covers a steady turn's axial trim. */
        if (bounded_diff * holding_diff > 0.0f && diff * holding_diff > 0.0f)
        {
            const float reserve = fminf(fabsf(holding_diff), fminf(fabsf(bounded_diff), fabsf(diff)));
            const float inner_min = holding_diff > 0.0f ? axial_lo[1] : axial_lo[0];

            cap = fminf(original, fmaxf(cap, 2.0f * inner_min + reserve));
        }
        const float left_min = fmaxf(axial_lo[0], cap - axial_hi[1]);
        const float left_max = fminf(axial_hi[0], cap - axial_lo[1]);
        const float conflict = fmaxf(2.0f * left_min - cap - diff, diff - (2.0f * left_max - cap));

        /* Only the differential lying outside the reachable common-force slice
         * competes with descent. A one-newton differential deficit can lift the
         * minimum feasible sum by one newton. The guard releases this priority. */
        if (conflict > 0.0f && cap < original)
        {
            const float sum = original - fminf(lower_share, 1.0f) * fminf(original - cap, conflict);
            const float left = Tinympc14_Clip(0.5f * (sum + diff), fmaxf(axial_lo[0], sum - axial_hi[1]),
                                              fminf(axial_hi[0], sum - axial_lo[1]));

            u_bare[4] = left;
            u_bare[5] = sum - left;
        }
    }
    for (i = 0; i < TINYMPC14_NU; i++)
    {
        tinympc14_async.u_prev[i] = u_bare[i];
        tinympc14_u[i] = u_bare[i] - offset[i];
    }
    return original - (u_bare[4] + u_bare[5]);
}

/* ---- output ---- */

static double Tinympc14_Output_Clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void Tinympc14_Output_Reset(tinympc14_comp_state_t *st)
{
    int i;
    int j;

    for (i = 0; i < TINYMPC14_OUTPUT_NITEM; i++)
    {
        for (j = 0; j < 2; j++)
        {
            st->v[i][j] = 0.0f;
            st->valid[i][j] = 0u;
            st->age[i][j] = 0u;
        }
    }
}

static void Tinympc14_Output_Put(tinympc14_comp_state_t *st, int item, int side, int ok, float val,
                                 float *out, unsigned char *status)
{
    if (ok)
    {
        st->v[item][side] = val;
        st->valid[item][side] = 1u;
        st->age[item][side] = 0u;
        *out = val;
        *status = TINYMPC14_OUTPUT_ST_FRESH;
        return;
    }

    if (st->age[item][side] < 255u)
    {
        st->age[item][side]++;
    }

    if (st->valid[item][side] && st->age[item][side] <= TINYMPC14_OUTPUT_HOLD_N)
    {
        *out = st->v[item][side];
        *status = TINYMPC14_OUTPUT_ST_HELD;
        return;
    }

    *out = 0.0f;
    *status = TINYMPC14_OUTPUT_ST_INVALID;
}

void Tinympc14_Output_Calc_Ex(float len_l, float len_r, float ang_l, float ang_r,
                              float v, float omega, tinympc14_comp_state_t *st, tinympc14_comp_t *c)
{
    float mgf[2];
    float fs[2];
    float fc[2];
    float l0;
    int ok_mgf[2];
    int ok_fs[2];
    int ok_fc;
    int i;

    for (i = 0; i < 2; i++)
    {
        float len = (i == 0) ? len_l : len_r;
        float ang = (i == 0) ? ang_l : ang_r;
        float theta0 = 0.0f;
        float l_d = 0.0f;

        mgf[i] = 0.0f;
        fs[i] = 0.0f;

        ok_mgf[i] = isfinite(len) && isfinite(ang);
        ok_fs[i] = isfinite(len);

        if (ok_mgf[i])
        {
#if defined(TINYMPC14_URDF_MODEL)
            const unsigned int count = sizeof(tinympc14_urdf_comp)/sizeof(tinympc14_urdf_comp[0]);
            const float length = Tinympc14_Clip(len,tinympc14_urdf_comp[0][0],tinympc14_urdf_comp[count-1u][0]);
            unsigned int row = 0u;
            while (row+2u < count && length >= tinympc14_urdf_comp[row+1u][0]) row++;
            const float fraction = (length-tinympc14_urdf_comp[row][0]) /
                                   (tinympc14_urdf_comp[row+1u][0]-tinympc14_urdf_comp[row][0]);
            theta0 = tinympc14_urdf_comp[row][1] + fraction*(tinympc14_urdf_comp[row+1u][1]-tinympc14_urdf_comp[row][1]);
            l_d = tinympc14_urdf_comp[row][2] + fraction*(tinympc14_urdf_comp[row+1u][2]-tinympc14_urdf_comp[row][2]);
#else
            Leg_Comp(len, &theta0, &l_d);
#endif
            mgf[i] = TINYMPC14_GRAV_FF * TINYMPC14_G * sinf(ang + theta0) * l_d;
        }

        if (ok_fs[i])
        {
            fs[i] = Leg_SpringF(len, 0.0f);
        }
    }

    fc[0] = 0.0f;
    fc[1] = 0.0f;
    ok_fc = isfinite(len_l) && isfinite(len_r) && isfinite(v) && isfinite(omega);
    if (ok_fc)
    {
        l0 = Tinympc14_Clip(0.5f * (len_l + len_r), TINYMPC14_L_MIN, TINYMPC14_L_MAX);
        fc[0] = Tinympc14_Clip(-TINYMPC14_M_SUP * v * omega * (l0 + TINYMPC14_H_OFF)
                               / TINYMPC14_ROLL_TRACK,
                               -TINYMPC14_ROLL_CLAMP, TINYMPC14_ROLL_CLAMP);
        fc[1] = -fc[0];
    }

    for (i = 0; i < TINYMPC14_OUTPUT_NU; i++)
    {
        c->ffwd[i] = 0.0f;
    }

    for (i = 0; i < 2; i++)
    {
        Tinympc14_Output_Put(st, TINYMPC14_OUTPUT_ITEM_MGF, i, ok_mgf[i], mgf[i],
                             &c->MgF[i], &c->st_MgF[i]);
        Tinympc14_Output_Put(st, TINYMPC14_OUTPUT_ITEM_FS, i, ok_fs[i], fs[i],
                             &c->Fs[i], &c->st_Fs[i]);
        Tinympc14_Output_Put(st, TINYMPC14_OUTPUT_ITEM_FC, i, ok_fc, fc[i],
                             &c->Fc[i], &c->st_Fc[i]);
    }
}

void Tinympc14_Output_Calc(float len_l, float len_r, float ang_l, float ang_r,
                           float v, float omega, tinympc14_comp_t *c)
{
    tinympc14_comp_state_t st;

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(len_l, len_r, ang_l, ang_r, v, omega, &st, c);
}

void Tinympc14_Output_Apply(const float u_mpc[TINYMPC14_OUTPUT_NU],const tinympc14_comp_t *c,
                    const float u_prev_applied[TINYMPC14_OUTPUT_NU],tinympc14_output_t *out)
{
    Tinympc14_Output_Apply_Period(u_mpc, c, u_prev_applied, TINYMPC14_NODE_TICKS, out);
}

void Tinympc14_Output_Apply_Period(const float u_mpc[TINYMPC14_OUTPUT_NU], const tinympc14_comp_t *c,
                                   const float u_prev_applied[TINYMPC14_OUTPUT_NU], uint32_t control_ms,
                                   tinympc14_output_t *out)
{
    int i;
    const float rate_scale = Tinympc14_Rate_Scale(control_ms);
    for(i=0;i<TINYMPC14_OUTPUT_NU;i++)
    {
        double prev=isfinite(u_prev_applied[i])?u_prev_applied[i]:0;
        double desired=isfinite(u_mpc[i])?u_mpc[i]:prev;
        double rate=rate_scale*tinympc14_du_rate[i];
        double lo,hi;
        prev=Tinympc14_Output_Clamp(prev,tinympc14_u_min[i],tinympc14_u_max[i]);
        if(i>=4 && isfinite(c->Fc[i-4])) desired+=c->Fc[i-4];
        lo=fmax(tinympc14_u_min[i],prev-rate);
        hi=fmin(tinympc14_u_max[i],prev+rate);
        out->u_applied[i]=(float)Tinympc14_Output_Clamp(desired,lo,hi);
    }
    for(i=0;i<2;i++)
    {
        out->T_w[i]=-out->u_applied[i];
        out->Tp[i]=out->u_applied[i+2]+(isfinite(c->MgF[i])?c->MgF[i]:0);
        out->F_motor[i]=out->u_applied[i+4]-(isfinite(c->Fs[i])?c->Fs[i]:0);
    }
}
