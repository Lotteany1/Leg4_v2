/**
 * @file  tinympc14_solver_host_test.c
 * @brief 主机对拍：固定向量 随机批量 补偿 异步 故障注入
 * @version  v0.2  2026-10-24
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "tinympc14_port_host.h"
#include "tinympc14.h"

#include "tinympc14_comp_vectors.h"
#include "tinympc14_rand_vectors.h"
#include "tinympc14_vectors.h"

#define TINYMPC14_TEST_TOL 2e-4f
#define TINYMPC14_TEST_EDGE_DUA 1.5e-4f

#ifndef TINYMPC14_TEST_INJECT_NAN
#define TINYMPC14_TEST_INJECT_NAN 0
#endif

static int tinympc14_test_fail;
static int tinympc14_test_edge;
static float tinympc14_test_max;
static float tinympc14_test_edge_max;

static int Tinympc14_Test_Max_Diff(const float *got, const float *exp, int n, float *m)
{
    int i;
    int bad = 0;
    float x = 0.0f;

    for (i = 0; i < n; i++)
    {
        if (!isfinite(got[i]) || !isfinite(exp[i]))
        {
            bad = 1;
            continue;
        }

        if (fabsf(got[i] - exp[i]) > x)
        {
            x = fabsf(got[i] - exp[i]);
        }
    }

    *m = x;

    return bad;
}

static int Tinympc14_Test_Is_Fail(const float *got, const float *exp, int n, float tol)
{
    float m = 0.0f;

    return Tinympc14_Test_Max_Diff(got, exp, n, &m) || m > tol;
}

static int Tinympc14_Test_Cmp(const char *tag, int idx, const float *got, const float *exp, int n, float tol)
{
    float m = 0.0f;
    int bad = Tinympc14_Test_Max_Diff(got, exp, n, &m);
    float score = bad ? INFINITY : m;

    if (score > tinympc14_test_max)
    {
        tinympc14_test_max = score;
    }

    if (bad || m > tol)
    {
        tinympc14_test_fail++;
        printf("  FAIL %s[%d] max|d|=%.3e%s\n", tag, idx, (double)m, bad ? " 非有限值" : "");
        return 1;
    }

    return 0;
}

static void Tinympc14_Test_Diff(const char *tag, int idx, const float *got, const float *exp, int n)
{
    (void)Tinympc14_Test_Cmp(tag, idx, got, exp, n, TINYMPC14_TEST_TOL);
}

static int Tinympc14_Test_Fixed(void)
{
    int i;
    int conv_mismatch = 0;

    printf("-- 固定向量 %d 例 --\n", (int)(sizeof(tinympc14_vectors) / sizeof(tinympc14_vectors[0])));

    for (i = 0; i < (int)(sizeof(tinympc14_vectors) / sizeof(tinympc14_vectors[0])); i++)
    {
        const tinympc14_vec_t *v = &tinympc14_vectors[i];
        float u[TINYMPC14_NU];
        int conv;

        Tinympc14_Init();
        (void)Tinympc14_Model_Update(v->lL, v->lR, 0.5f * (v->x0[4] + v->x0[6]));
        conv = Tinympc14_Solve(v->x0, v->xref, v->fed, v->ffwd, v->u_prev, u);

#if TINYMPC14_TEST_INJECT_NAN
        u[0] = NAN;
#endif
        Tinympc14_Test_Diff("fixed", i + 1, u, v->u_expect, TINYMPC14_NU);

        if (conv != v->conv_expect)
        {
            conv_mismatch++;
            printf("  NOTE #%d conv=%d expect=%d\n", i + 1, conv, v->conv_expect);
        }
    }

    printf("   收敛标记不一致 %d 例\n", conv_mismatch);

    return conv_mismatch;
}

static int Tinympc14_Test_Rand(void)
{
    int i;
    int conv_mismatch = 0;
    int n = (int)(sizeof(tinympc14_rand_vectors) / sizeof(tinympc14_rand_vectors[0]));

    printf("-- 随机批量 %d 例 --\n", n);
    Tinympc14_Init();

    for (i = 0; i < n; i++)
    {
        const tinympc14_rand_vec_t *v = &tinympc14_rand_vectors[i];
        float u[TINYMPC14_NU];
        float m = 0.0f;
        int edge;
        int bad;
        int conv = Tinympc14_Solve(v->x0, v->xref, v->fed, v->ffwd, v->u_prev, u);

        bad = Tinympc14_Test_Max_Diff(u, v->u_expect, TINYMPC14_NU, &m);

        edge = 0; /* proximity to a tolerance cannot excuse a wrong actuator output */

        if (bad || m > tinympc14_test_max)
        {
            tinympc14_test_max = bad ? INFINITY : m;      /* 全局最大，不剔边缘例 */
        }

        if (!bad && m <= TINYMPC14_TEST_TOL)
        {
            ;
        }
        else if (edge && !bad)
        {
            tinympc14_test_edge++;
            printf("  EDGE rand[%d] max|d|=%.3e dua=%.3e it=%d conv=%d/%d\n", i + 1, (double)m,
                   (double)tinympc14_status.dua, tinympc14_status.iters, conv, v->conv_expect);
        }
        else
        {
            tinympc14_test_fail++;
            printf("  FAIL rand[%d] max|d|=%.3e 非有限=%d dua=%.3e it=%d\n", i + 1, (double)m, bad,
                   (double)tinympc14_status.dua, tinympc14_status.iters);
        }

        if (conv != v->conv_expect)
        {
            conv_mismatch++;
        }
    }

    printf("   收敛标记不一致 %d 例\n", conv_mismatch);

    return conv_mismatch;
}

static void Tinympc14_Test_Comp_Status(const char *tag, int idx, const tinympc14_comp_t *c, const tinympc14_comp_vec_t *v)
{
    if (c->st_MgF[0] != v->st[0][0] || c->st_MgF[1] != v->st[0][1]
        || c->st_Fs[0] != v->st[1][0] || c->st_Fs[1] != v->st[1][1]
        || c->st_Fc[0] != v->st[2][0] || c->st_Fc[1] != v->st[2][1])
    {
        tinympc14_test_fail++;
        printf("  FAIL %s[%d] 单笔状态 got {MgF %u,%u}{Fs %u,%u}{Fc %u,%u} exp {MgF %u,%u}{Fs %u,%u}{Fc %u,%u}\n",
               tag, idx, c->st_MgF[0], c->st_MgF[1], c->st_Fs[0], c->st_Fs[1],
               c->st_Fc[0], c->st_Fc[1], v->st[0][0], v->st[0][1], v->st[1][0],
               v->st[1][1], v->st[2][0], v->st[2][1]);
    }
}

static int Tinympc14_Test_Comp(void)
{
    int i;
    int n = (int)(sizeof(tinympc14_comp_vectors) / sizeof(tinympc14_comp_vectors[0]));

    printf("-- 补偿向量 %d 段 --\n", n);

    for (i = 0; i < n; i++)
    {
        const tinympc14_comp_vec_t *v = &tinympc14_comp_vectors[i];
        tinympc14_comp_state_t st;
        tinympc14_comp_t c;
        tinympc14_comp_t cs;
        int k;

        Tinympc14_Output_Reset(&st);

        for (k = 0; k < v->n_step; k++)
        {
            Tinympc14_Output_Calc_Ex(v->in[k][0], v->in[k][1], v->in[k][2], v->in[k][3],
                              v->in[k][4], v->in[k][5], &st, &c);
        }

        Tinympc14_Test_Diff("MgF", i + 1, c.MgF, v->MgF, 2);
        Tinympc14_Test_Diff("Fs", i + 1, c.Fs, v->Fs, 2);
        Tinympc14_Test_Diff("Fc", i + 1, c.Fc, v->Fc, 2);
        Tinympc14_Test_Diff("ffwd", i + 1, c.ffwd, v->ffwd, TINYMPC14_OUTPUT_NU);
        Tinympc14_Test_Comp_Status("comp", i + 1, &c, v);

        if (v->n_step == 1)
        {
            Tinympc14_Output_Calc(v->in[0][0], v->in[0][1], v->in[0][2], v->in[0][3],
                           v->in[0][4], v->in[0][5], &cs);
            Tinympc14_Test_Diff("noState-MgF", i + 1, cs.MgF, v->MgF, 2);
            Tinympc14_Test_Diff("noState-Fs", i + 1, cs.Fs, v->Fs, 2);
            Tinympc14_Test_Diff("noState-Fc", i + 1, cs.Fc, v->Fc, 2);
        }

#if TINYMPC14_TEST_INJECT_NAN
        c.Fs[0] = NAN;
        Tinympc14_Test_Diff("inject-Fs", i + 1, c.Fs, v->Fs, 2);
#endif
    }

    return 0;
}

static void Tinympc14_Test_Comp_Isolation(void)
{
    tinympc14_comp_state_t st;
    tinympc14_comp_t a;
    tinympc14_comp_t b;
    float one_got[1];
    float one_exp[1];

    printf("-- 补偿失效隔离（无关传感器 / 单侧几何）--\n");

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, 0.5f, 1.0f, &st, &a);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, 0.5f, NAN, &st, &b);
    Tinympc14_Test_Cmp("omegaNaN-Fs", 0, b.Fs, a.Fs, 2, 1e-6f);
    Tinympc14_Test_Cmp("omegaNaN-MgF", 0, b.MgF, a.MgF, 2, 1e-6f);
    Tinympc14_Test_Cmp("omegaNaN-Fc-held", 0, b.Fc, a.Fc, 2, 1e-6f);

    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, NAN, 1.0f, &st, &a);
    if (a.st_Fc[0] != TINYMPC14_OUTPUT_ST_HELD || a.st_Fs[0] != TINYMPC14_OUTPUT_ST_FRESH)
    {
        tinympc14_test_fail++;
        printf("  FAIL vNaN 状态 Fc=%u Fs=%u\n", a.st_Fc[0], a.st_Fs[0]);
    }

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, 0.5f, 1.0f, &st, &a);
    Tinympc14_Output_Calc_Ex(0.27f, NAN, 0.0f, 0.0f, 0.5f, 1.0f, &st, &b);
    one_got[0] = b.Fs[0];
    one_exp[0] = a.Fs[0];
    Tinympc14_Test_Cmp("lenrNaN-Fs_l", 0, one_got, one_exp, 1, 1e-6f);
    one_got[0] = b.MgF[0];
    one_exp[0] = a.MgF[0];
    Tinympc14_Test_Cmp("lenrNaN-MgF_l", 0, one_got, one_exp, 1, 1e-6f);
    one_got[0] = b.Fs[1];
    one_exp[0] = a.Fs[1];
    Tinympc14_Test_Cmp("lenrNaN-Fs_r-held", 0, one_got, one_exp, 1, 1e-6f);
    one_got[0] = b.Fc[0];
    one_exp[0] = a.Fc[0];
    Tinympc14_Test_Cmp("lenrNaN-Fc-held", 0, one_got, one_exp, 1, 1e-6f);
    if (b.st_Fs[0] != TINYMPC14_OUTPUT_ST_FRESH || b.st_Fs[1] != TINYMPC14_OUTPUT_ST_HELD)
    {
        tinympc14_test_fail++;
        printf("  FAIL lenrNaN 状态 Fs_l=%u Fs_r=%u\n", b.st_Fs[0], b.st_Fs[1]);
    }
    one_exp[0] = a.Fs[0];
    if (!isfinite(b.Fs[0]) || fabsf(b.Fs[0] - one_exp[0]) > 1e-6f || b.Fs[0] <= 0.0f)
    {
        tinympc14_test_fail++;
        printf("  FAIL lenrNaN 左腿弹簧力被牵连 %.4f\n", (double)b.Fs[0]);
    }

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(NAN, 0.27f, 0.0f, 0.0f, 0.5f, 1.0f, &st, &b);
    one_got[0] = b.Fs[1];
    one_exp[0] = a.Fs[1];
    Tinympc14_Test_Cmp("lenlNaN-Fs_r", 0, one_got, one_exp, 1, 1e-6f);
    one_got[0] = b.MgF[1];
    one_exp[0] = a.MgF[1];
    Tinympc14_Test_Cmp("lenlNaN-MgF_r", 0, one_got, one_exp, 1, 1e-6f);
    if (b.st_Fs[0] != TINYMPC14_OUTPUT_ST_INVALID || b.st_Fs[1] != TINYMPC14_OUTPUT_ST_FRESH
        || b.st_MgF[1] != TINYMPC14_OUTPUT_ST_FRESH)
    {
        tinympc14_test_fail++;
        printf("  FAIL lenlNaN 状态 Fs_l=%u Fs_r=%u MgF_r=%u\n", b.st_Fs[0], b.st_Fs[1], b.st_MgF[1]);
    }

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, NAN, 0.0f, 0.5f, 1.0f, &st, &b);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, 0.5f, 1.0f, &st, &a);
    one_got[0] = b.Fs[0];
    one_exp[0] = a.Fs[0];
    Tinympc14_Test_Cmp("anglNaN-Fs_l", 0, one_got, one_exp, 1, 1e-6f);
    one_got[0] = b.Fc[0];
    one_exp[0] = a.Fc[0];
    Tinympc14_Test_Cmp("anglNaN-Fc", 0, one_got, one_exp, 1, 1e-6f);
    if (b.st_MgF[0] != TINYMPC14_OUTPUT_ST_INVALID || b.st_Fs[0] != TINYMPC14_OUTPUT_ST_FRESH
        || b.st_Fc[0] != TINYMPC14_OUTPUT_ST_FRESH)
    {
        tinympc14_test_fail++;
        printf("  FAIL anglNaN 状态 MgF_l=%u Fs_l=%u Fc=%u\n", b.st_MgF[0], b.st_Fs[0], b.st_Fc[0]);
    }

    Tinympc14_Output_Reset(&st);
    Tinympc14_Output_Calc_Ex(0.27f, 0.27f, 0.0f, 0.0f, 0.5f, NAN, &st, &b);
    if (b.st_Fc[0] != TINYMPC14_OUTPUT_ST_INVALID || b.Fc[0] != 0.0f || b.Fc[1] != 0.0f)
    {
        tinympc14_test_fail++;
        printf("  FAIL 首拍无历史时 Fc 必须报无效而不是沿用 %.3f\n", (double)b.Fc[0]);
    }
}

static int Tinympc14_Test_Nan(void)
{
    const tinympc14_vec_t *v = &tinympc14_vectors[0];
    float x0[TINYMPC14_NX];
    float u[TINYMPC14_NU];
    unsigned n0;
    int conv;

    printf("-- NaN 入口 --\n");
    Tinympc14_Init();
    memcpy(x0, v->x0, sizeof(x0));
    x0[0] = NAN;
    n0 = tinympc14_counters.nan;
    conv = Tinympc14_Solve(x0, v->xref, v->fed, v->ffwd, v->u_prev, u);

    if (conv != 0 || tinympc14_counters.nan != n0 + 1u)
    {
        tinympc14_test_fail++;
        printf("  FAIL NaN conv=%d nan=%u\n", conv, (unsigned)tinympc14_counters.nan);
        return 1;
    }

    return 0;
}

static int Tinympc14_Test_Async(void)
{
    const tinympc14_vec_t *v = &tinympc14_vectors[0];
    float seed[TINYMPC14_NU];
    float u_async[TINYMPC14_NU];
    float u_sync[TINYMPC14_NU];
    int k;

    printf("-- 异步 1 ms 控制拍 / 2 ms 预测节点 --\n");

    Tinympc14_Init();
    Tinympc14_Seed(seed);
    Tinympc14_Port_Host_Set_Tick(0u);
    Tinympc14_Async_Input(v->x0, v->xref, v->fed, v->ffwd, v->lL, v->lR);
    Tinympc14_Async_Solve();
    Tinympc14_Async_Step_Period(1u);

    memcpy(u_async, (const float *)tinympc14_u, sizeof(u_async));

    for (k = 1; k < 14; k++)
    {
        Tinympc14_Port_Host_Set_Tick((uint32_t)k);
        Tinympc14_Async_Step_Period(1u);
    }

    if (tinympc14_counters.fallback != 0u)
    {
        tinympc14_test_fail++;
        printf("  FAIL async 前 7 拍回退 %u\n", tinympc14_counters.fallback);
    }

    Tinympc14_Port_Host_Set_Tick(14u);
    Tinympc14_Async_Step_Period(1u);

    if (tinympc14_counters.stale != 1u || tinympc14_counters.fallback != 1u
        || tinympc14_counters.node_repeat != 7u)
    {
        tinympc14_test_fail++;
        printf("  FAIL async 超龄 stale=%u fallback=%u repeat=%u\n",
               tinympc14_counters.stale, tinympc14_counters.fallback,
               tinympc14_counters.node_repeat);
    }

    Tinympc14_Init();
    (void)Tinympc14_Model_Update(v->lL, v->lR, 0.5f * (v->x0[4] + v->x0[6]));
    (void)Tinympc14_Solve(v->x0, v->xref, v->fed, v->ffwd, seed, u_sync);
    Tinympc14_Test_Diff("async0", 1, u_async, u_sync, TINYMPC14_NU);

    /* Fixed height step from minimum support has a rate-saturated plan. */
    v = &tinympc14_vectors[5];
    Tinympc14_Init();
    Tinympc14_Port_Host_Set_Tick(0u);
    Tinympc14_Async_Applied(v->u_prev);
    Tinympc14_Async_Input(v->x0, v->xref, v->fed, v->ffwd, v->lL, v->lR);
    Tinympc14_Async_Solve();
    for (k = 0; k < 2; k++)
    {
        float expected[2], actual[2];
        Tinympc14_Port_Host_Set_Tick((uint32_t)k);
        Tinympc14_Async_Step_Period(1u);
        for (int side = 0; side < 2; side++)
        {
            const int j = side + 4;
            expected[side] = v->u_prev[j] + 0.5f * (float)(k+1) * tinympc14_du_rate[j];
            actual[side] = tinympc14_u[j];
        }
        Tinympc14_Test_Diff("async-force-slew-1ms", k, actual, expected, 2);
    }

    return 0;
}

static void cancel_and_resume(void)
{
    Tinympc14_Async_Invalidate();
    Tinympc14_Async_Run();
}

static void expire_on_finish(void)
{
    Tinympc14_Port_Host_Set_Tick(7u);
}

static void arm_finish_expiry(void)
{
    Tinympc14_Port_Host_On_Tick(expire_on_finish);
}

static void check_solve_accounting(void)
{
    unsigned finished = tinympc14_counters.commit + tinympc14_counters.reject_start_age +
                        tinympc14_counters.reject_input + tinympc14_counters.reject_convergence +
                        tinympc14_counters.reject_epoch + tinympc14_counters.reject_finish_age;
    if (finished != tinympc14_counters.solve_attempt)
    {
        tinympc14_test_fail++;
        printf("FAIL async attempted=%u finished=%u\n", tinympc14_counters.solve_attempt, finished);
    }
}

/* Model preparation spans tick wrap. The estimator supplies a different
 * measurement, command, and previous input while the factors are built. */
static float refresh_left;
static void inject_refresh_sample(void)
{
    const tinympc14_vec_t *v = &tinympc14_vectors[0];
    float x[TINYMPC14_NX], ref[TINYMPC14_NX];
    memcpy(x, v->x0, sizeof(x));
    memcpy(ref, v->xref, sizeof(ref));
    x[10] += 0.03f;
    ref[12] -= 0.04f;
    Tinympc14_Async_Applied(v->u_prev);
    Tinympc14_Async_Input_At(x, ref, v->fed, v->ffwd, refresh_left, 0.24f, 6u);
    Tinympc14_Port_Host_On_Tick(expire_on_finish);
}

static void Tinympc14_Test_Model_Refresh_Sample(void)
{
    const tinympc14_vec_t *v = &tinympc14_vectors[0];
    float expected[TINYMPC14_NUU];
    int matching;
    /* Reference solve starts directly on the new control beat with the same
     * scheduling point as the factors built by the asynchronous path. */
    Tinympc14_Init();
    Tinympc14_Port_Host_Set_Tick(7u);
    refresh_left = 0.24f;
    inject_refresh_sample();
    Tinympc14_Async_Solve();
    memcpy(expected, Tinympc14_Sequence(), sizeof(expected));
    if (tinympc14_counters.commit != 1u) ++tinympc14_test_fail;
    for (matching = 0; matching <= 1; ++matching)
    {
        Tinympc14_Init();
        refresh_left = matching ? 0.242f : 0.30f;
        Tinympc14_Port_Host_Set_Tick(UINT32_MAX - 6u);
        Tinympc14_Async_Input(v->x0, v->xref, v->fed, v->ffwd, 0.24f, 0.24f);
        Tinympc14_Port_Host_On_Tick(inject_refresh_sample);
        Tinympc14_Async_Solve();
        if (tinympc14_counters.lpv_updates != 1u || tinympc14_counters.commit != (unsigned)matching ||
            tinympc14_counters.reject_finish_age != (unsigned)!matching ||
            tinympc14_counters.solve_age_ms != (matching ? 1u : 14u))
        {
            ++tinympc14_test_fail;
            printf("FAIL sample during model refresh, matching=%d\n", matching);
        }
        if (matching)
            Tinympc14_Test_Cmp("new sample after LPV refresh", 0, Tinympc14_Sequence(),
                               expected, TINYMPC14_NUU, TINYMPC14_TEST_TOL);
        check_solve_accounting();
    }
}

static void Tinympc14_Test_Async_Regression(void)
{
    const tinympc14_vec_t *v=&tinympc14_vectors[0];
    float ff[TINYMPC14_NU],us[TINYMPC14_NU],uf[TINYMPC14_NU];
    float expected[TINYMPC14_NU],prev[TINYMPC14_NU],got[TINYMPC14_NU];
    float xn[TINYMPC14_NX];
    unsigned before; int i;
    printf("-- async recovery / offset / cancellation regressions --\n");
    Tinympc14_Init(); Tinympc14_Port_Host_Set_Tick(0);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Async_Solve();
    Tinympc14_Affine_Pub(v->xref,v->fed,v->ffwd,us,uf);
    for(i=0;i<TINYMPC14_NU;i++) expected[i]=Tinympc14_Sequence()[TINYMPC14_NU+i]+uf[i];
    Tinympc14_Async_Step();
    for(i=0;i<TINYMPC14_NU;i++) prev[i]=tinympc14_u[i];
    memcpy(ff,v->ffwd,sizeof(ff)); ff[4]+=2; ff[5]-=2;
    Tinympc14_Port_Host_Set_Tick(2);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,ff,v->lL,v->lR);
    Tinympc14_Async_Step();
    for(i=0;i<TINYMPC14_NU;i++)
    {
        expected[i]=Tinympc14_Clip(expected[i],fmaxf(tinympc14_u_min[i],prev[i]-tinympc14_du_rate[i]),
                                    fminf(tinympc14_u_max[i],prev[i]+tinympc14_du_rate[i]));
        got[i]=tinympc14_u[i];
    }
    Tinympc14_Test_Diff("committed-offset",0,got,expected,TINYMPC14_NU);
    Tinympc14_Async_Step();
    if(tinympc14_counters.node_repeat!=1) { tinympc14_test_fail++; printf("FAIL repeated node detection\n"); }

    /* A transient solver failure must retry without an explicit Run. */
    Tinympc14_Init(); Tinympc14_Port_Host_Set_Tick(0);
    memcpy(xn,v->x0,sizeof(xn)); xn[10]=2;
    Tinympc14_Async_Input(xn,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Async_Solve();
    if(tinympc14_status.converged || tinympc14_counters.reject_convergence != 1u)
    { tinympc14_test_fail++; printf("FAIL stress convergence diagnostic\n"); }
    for(i=0;i<4;i++)
    {
        Tinympc14_Port_Host_Set_Tick((uint32_t)(4+4*i));
        Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
        Tinympc14_Async_Solve();
    }
    if(tinympc14_counters.commit==0) { tinympc14_test_fail++; printf("FAIL solver did not recover\n"); }
    before=tinympc14_counters.commit;
    xn[0]=NAN;
    Tinympc14_Port_Host_Set_Tick(20);
    Tinympc14_Async_Input(xn,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Async_Solve();
    if (tinympc14_counters.reject_input != 1u)
    { tinympc14_test_fail++; printf("FAIL invalid input diagnostic\n"); }
    Tinympc14_Async_Step();
    Tinympc14_Port_Host_Set_Tick(22);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Async_Solve();
    if(tinympc14_counters.commit<=before) { tinympc14_test_fail++; printf("FAIL NaN recovery\n"); }
    check_solve_accounting();

    /* Explicit cancellation during a solve must prevent its later publication. */
    Tinympc14_Init(); Tinympc14_Port_Host_Set_Tick(0);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Port_Host_On_Tick(cancel_and_resume);
    Tinympc14_Async_Solve();
    if(tinympc14_counters.commit!=0 || tinympc14_counters.reject_epoch!=1u)
    { tinympc14_test_fail++; printf("FAIL cancelled solve diagnostic\n"); }
    check_solve_accounting();
    Tinympc14_Init(); Tinympc14_Port_Host_Set_Tick(0);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Port_Host_Set_Tick(16); Tinympc14_Async_Solve();
    if(tinympc14_counters.commit!=0 || tinympc14_counters.reject_start_age!=1u ||
       tinympc14_counters.solve_age_ms!=16u)
    { tinympc14_test_fail++; printf("FAIL stale snapshot diagnostic\n"); }
    check_solve_accounting();

    /* A timely snapshot converges at exactly age 14 ms, crossing tick wrap.
     * No published sequence has expired in the consumer. */
    Tinympc14_Init();
    Tinympc14_Async_Solve();
    if (tinympc14_counters.solve_attempt != 0u || tinympc14_counters.solve_age_ms != 0u)
    { tinympc14_test_fail++; printf("FAIL idle solve counted or age not reset\n"); }
    Tinympc14_Port_Host_Set_Tick(UINT32_MAX - 6u);
    Tinympc14_Async_Input(v->x0,v->xref,v->fed,v->ffwd,v->lL,v->lR);
    Tinympc14_Port_Host_On_Tick(arm_finish_expiry);
    Tinympc14_Async_Solve();
    Tinympc14_Async_Step_Period(1u);
    if (!tinympc14_status.converged || tinympc14_counters.reject_finish_age != 1u ||
        tinympc14_counters.solve_age_ms != 14u || tinympc14_counters.commit != 0u ||
        tinympc14_counters.fallback != 1u || tinympc14_counters.stale != 0u)
    { tinympc14_test_fail++; printf("FAIL converged over-age publication diagnostic\n"); }
    check_solve_accounting();
}

static void Tinympc14_Test_External_Comp(void)
{
    const tinympc14_vec_t *v=&tinympc14_vectors[0];
    tinympc14_comp_state_t st; tinympc14_comp_t c; tinympc14_output_t out;
    float expected[TINYMPC14_NU],got[TINYMPC14_NU];
    float total_got[2],total_exp[2],hip_got[2],hip_exp[2],wheel_exp[2];
    float fs_ok[2];
    int i;
    printf("-- external compensation and applied-input feedback --\n");
    Tinympc14_Init(); Tinympc14_Seed(expected);
    for(i=0;i<TINYMPC14_NU;i++) got[i]=tinympc14_u[i];
    Tinympc14_Test_Diff("startup-seed",0,got,expected,TINYMPC14_NU);
    Tinympc14_Output_Reset(&st);
    /* Exercise the 6 N/node slew limit on both legs with the 777 support mass. */
    Tinympc14_Output_Calc_Ex(.27f,.27f,0,0,1,2,&st,&c);
    Tinympc14_Output_Apply(v->u_prev,&c,v->u_prev,&out);
    for(i=0;i<2;i++)
    {
        total_got[i]=out.F_motor[i]+c.Fs[i];
        total_exp[i]=out.u_applied[i+4];
        hip_got[i]=out.Tp[i]-c.MgF[i];
        hip_exp[i]=out.u_applied[i+2];
        wheel_exp[i]=-v->u_prev[i];
        fs_ok[i]=c.Fs[i];
        expected[i+4]=v->u_prev[i+4]+(i==0?-6.0f:6.0f);
    }
    Tinympc14_Test_Cmp("total-force-Fmotor+Fs",0,total_got,total_exp,2,2e-5f);
    Tinympc14_Test_Cmp("hip-torque-Tp-MgF",0,hip_got,hip_exp,2,2e-5f);
    Tinympc14_Test_Cmp("wheel-sign",0,out.T_w,wheel_exp,2,1e-6f);
    for(i=0;i<4;i++) expected[i]=v->u_prev[i];
    Tinympc14_Test_Diff("post-compensation-slew",0,out.u_applied,expected,TINYMPC14_NU);
    Tinympc14_Output_Apply_Period(v->u_prev,&c,v->u_prev,1u,&out);
    expected[4]=v->u_prev[4]-0.5f*tinympc14_du_rate[4];
    expected[5]=v->u_prev[5]+0.5f*tinympc14_du_rate[5];
    Tinympc14_Test_Diff("post-compensation-slew-1ms",0,out.u_applied,expected,TINYMPC14_NU);
    for(i=0;i<TINYMPC14_NU;i++) if(c.ffwd[i]!=0) tinympc14_test_fail++;

    Tinympc14_Output_Calc_Ex(.27f,.27f,0,0,1,NAN,&st,&c);
    if(c.st_Fc[0]!=TINYMPC14_OUTPUT_ST_HELD||c.st_Fs[0]!=TINYMPC14_OUTPUT_ST_FRESH)
    { tinympc14_test_fail++; printf("FAIL omega NaN invalidated an unrelated compensation\n"); }
    Tinympc14_Output_Apply(v->u_prev,&c,v->u_prev,&out);
    for(i=0;i<2;i++)
    {
        total_got[i]=out.F_motor[i]+c.Fs[i];
        total_exp[i]=out.u_applied[i+4];
        hip_got[i]=out.Tp[i]-c.MgF[i];
        hip_exp[i]=out.u_applied[i+2];
    }
    Tinympc14_Test_Cmp("omegaNaN-total-force",0,total_got,total_exp,2,2e-5f);
    Tinympc14_Test_Cmp("omegaNaN-hip-torque",0,hip_got,hip_exp,2,2e-5f);
    Tinympc14_Test_Cmp("omegaNaN-spring-kept",0,c.Fs,fs_ok,2,1e-6f);
    for(i=0;i<2;i++)
    {
        if(!isfinite(out.F_motor[i]) || fabsf(out.F_motor[i]-out.u_applied[i+4])<1.0f)
        { tinympc14_test_fail++; printf("FAIL motor delivers the whole total force while the spring is live\n"); }
    }

    Tinympc14_Init(); Tinympc14_Async_Applied(out.u_applied);
    Tinympc14_Async_Invalidate(); Tinympc14_Async_Step();
    for(i=0;i<TINYMPC14_NU;i++) got[i]=tinympc14_u[i];
    Tinympc14_Test_Diff("applied-feedback",0,got,out.u_applied,TINYMPC14_NU);
}

static int Tinympc14_Test_Guard(void)
{
    float a[4]={1.f,2.f,3.f,4.f};
    float b[4]={1.f,2.f,3.f,4.f};
    float nan_res[4]={1.f,2.f,NAN,4.f};
    float nan_exp[4]={1.f,2.f,NAN,4.f};
    float inf_res[4]={1.f,2.f,INFINITY,4.f};
    float big[4]={1.f,2.f,3.5f,4.f};
    float close[4]={1.f,2.f,3.00001f,4.f};
    int bad=0;

    printf("-- 比较函数 NaN 漏检自检 --\n");
    if(Tinympc14_Test_Is_Fail(a,b,4,TINYMPC14_TEST_TOL)) bad=1;
    if(!Tinympc14_Test_Is_Fail(nan_res,b,4,TINYMPC14_TEST_TOL)) bad=1;
    if(!Tinympc14_Test_Is_Fail(a,nan_exp,4,TINYMPC14_TEST_TOL)) bad=1;
    if(!Tinympc14_Test_Is_Fail(inf_res,b,4,TINYMPC14_TEST_TOL)) bad=1;
    if(!Tinympc14_Test_Is_Fail(big,b,4,TINYMPC14_TEST_TOL)) bad=1;
    if(Tinympc14_Test_Is_Fail(close,b,4,TINYMPC14_TEST_TOL)) bad=1;
    if(bad)
    {
        tinympc14_test_fail++;
        printf("  FAIL 比较函数未拒绝非有限值或超差\n");
    }
    return bad;
}

int main(void)
{
    int conv_bad = 0;

    printf("Leg4 solver host test\n");

    conv_bad += Tinympc14_Test_Guard();
    conv_bad += Tinympc14_Test_Fixed();
    conv_bad += Tinympc14_Test_Rand();
    (void)Tinympc14_Test_Comp();
    Tinympc14_Test_Comp_Isolation();
    (void)Tinympc14_Test_Nan();
    (void)Tinympc14_Test_Async();
    Tinympc14_Test_Async_Regression();
    Tinympc14_Test_Model_Refresh_Sample();
    Tinympc14_Test_External_Comp();

    printf("\nmax|d| = %.3e  失败项 = %d  边缘项 = %d\n", (double)tinympc14_test_max,
           tinympc14_test_fail, tinympc14_test_edge);

    if (tinympc14_test_fail != 0 || tinympc14_test_max > TINYMPC14_TEST_TOL || conv_bad!=0)
    {
        printf("RESULT FAIL\n");
        return 1;
    }

    if (tinympc14_test_edge != 0)
    {
        printf("RESULT PASS 边缘项 %d 例停在 dua_tol 邻域 见上\n", tinympc14_test_edge);
        return 0;
    }

    printf("RESULT PASS\n");

    return 0;
}
