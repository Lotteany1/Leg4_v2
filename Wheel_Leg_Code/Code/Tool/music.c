/**
 * @file  music.c
 * @brief 蜂鸣器音符播放引擎
 * @version  v1.1  2026-10-26
 */
#include "music.h"
#include "songs.h"
#include "system_key.h"
#if APP_USE_MUSIC
#include "tim.h"

#define MUSIC_TIM_CLK_HZ 275000000UL /* Hz，TIM12 时钟 */
#define MUSIC_TIM_PSC    32UL        /* 分频比 33，计数频率约 8.33MHz 16 位 ARR 最低约 127Hz 盖住 C3 */

/* 十二平均律查表 f(n) = 440*2^((n-69)/12)
   下标即 MIDI 音高 60(C4)=262 69(A4)=440 72(C5)=523 */
const uint16_t Music_NoteHz[MUSIC_NOTE_NUM] =
    {
        8U,
        9U,
        9U,
        10U,
        10U,
        11U,
        12U,
        12U,
        13U,
        14U,
        15U,
        15U,
        16U,
        17U,
        18U,
        19U,
        21U,
        22U,
        23U,
        24U,
        26U,
        28U,
        29U,
        31U,
        33U,
        35U,
        37U,
        39U,
        41U,
        44U,
        46U,
        49U,
        52U,
        55U,
        58U,
        62U,
        65U,
        69U,
        73U,
        78U,
        82U,
        87U,
        92U,
        98U,
        104U,
        110U,
        117U,
        123U,
        131U,
        139U,
        147U,
        156U,
        165U,
        175U,
        185U,
        196U,
        208U,
        220U,
        233U,
        247U,
        262U,
        277U,
        294U,
        311U,
        330U,
        349U,
        370U,
        392U,
        415U,
        440U,
        466U,
        494U,
        523U,
        554U,
        587U,
        622U,
        659U,
        698U,
        740U,
        784U,
        831U,
        880U,
        932U,
        988U,
        1047U,
        1109U,
        1175U,
        1245U,
        1319U,
        1397U,
        1480U,
        1568U,
        1661U,
        1760U,
        1865U,
        1976U,
        2093U,
        2217U,
        2349U,
        2489U,
        2637U,
        2794U,
        2960U,
        3136U,
        3322U,
        3520U,
        3729U,
        3951U,
        4186U,
        4435U,
        4699U,
        4978U,
        5274U,
        5588U,
        5920U,
        6272U,
        6645U,
        7040U,
        7459U,
        7902U,
        8372U,
        8870U,
        9397U,
        9956U,
        10548U,
        11175U,
        11840U,
        12544U,
};

static const Music_Song_t *s_song = NULL; /* 当前曲目 */
static uint16_t s_pos             = 0;    /* 当前音符下标 */
static uint32_t s_noteMs          = 0;    /* ms，当前音符时长 */
static uint32_t s_t0              = 0;    /* tick，当前音符起始 */
static uint8_t s_playing          = 0;    /* 播放标志 */

// 停止发声只置占空比 0 不动定时器 无开关声
static void music_tone_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
}

// 按 Hz 发声占空比 50% 0 或超蜂鸣器响应范围视为休止
void Music_Tone(uint16_t hz)
{
    uint32_t arr;

    if ((hz == 0U) || (hz > MUSIC_MAX_HZ))
    {
        music_tone_off();
        return;
    }

    // 1. ARR = 计数频率 / 音高频率 - 1, 计数频率 = 275MHz / 33
    arr = (MUSIC_TIM_CLK_HZ / (MUSIC_TIM_PSC + 1UL)) / (uint32_t)hz;
    if ((arr == 0U) || (arr > 65536UL)) /* 超出 16 位 ARR 范围 低于约 127Hz */
    {
        music_tone_off();
        return;
    }
    arr -= 1UL;

    // 2. ARR 与 CCR 均预装载 每个 PWM 周期结束的更新事件后生效 切换平滑
    __HAL_TIM_SET_AUTORELOAD(&htim12, (uint16_t)arr);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (uint16_t)((arr + 1UL) >> 1UL));
}

// 当前下标发声并算出时值
static void music_play_pos(void)
{
    const Music_Note_t *ev;
    uint32_t durMs;

    ev = &s_song->notes[s_pos];

    if ((ev->note == MUSIC_NOTE_REST) || (ev->note >= MUSIC_NOTE_NUM))
    {
        music_tone_off(); /* 休止或非法音高 静音 */
    }
    else
    {
        // 1. 整体移调 查表等价于频率 x 2^(shift/12) 无累积误差
        uint8_t note = (uint8_t)(ev->note + MUSIC_SHIFT_SEMIS);
        if (note >= MUSIC_NOTE_NUM)
        {
            music_tone_off(); /* 移调后超频率表 不可发声 */
        }
        else
        {
            Music_Tone(Music_NoteHz[note]);
        }
    }

    // 2. 时值 ms = dur * 60000 / (bpm * 8) 时值单位 1/32 音符 四分音符 = 8 单位
    durMs    = ((uint32_t)ev->dur * 60000UL) / ((uint32_t)(s_song->bpm ? s_song->bpm : 120U) * 8UL);
    s_noteMs = (durMs > 0UL) ? durMs : 1UL;
}

// 蜂鸣器 PWM 初始化 设置分频与静音占空比
void Music_Init(void)
{
    // 1. 改预分频扩低音范围 先装影子寄存器再启动 免分频突变
    __HAL_TIM_SET_PRESCALER(&htim12, MUSIC_TIM_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim12, 1000U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);

    // 2. 本工程 H7 HAL 无 __HAL_TIM_GENERATE_UPDATE 宏 EGR 置 UG 产生软件更新事件
    htim12.Instance->EGR = TIM_EGR_UG;
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    s_playing = 0;
    s_song    = NULL;
}

// 按曲目编号开始播放 首音立即装载
void Music_Play(uint8_t idx)
{
    Music_Stop(); /* 先静音清状态 支持播放中切歌 */

    s_song = Music_GetSong(idx);
    if ((s_song == NULL) || (s_song->len == 0U))
    {
        return;
    }

    s_pos     = 0;
    s_playing = 1;
    s_t0      = HAL_GetTick();
    music_play_pos();
}

// 播放状态清零 蜂鸣器静音
void Music_Stop(void)
{
    s_playing = 0;
    music_tone_off();
}

// 播放标志只读
uint8_t Music_IsPlaying(void)
{
    return s_playing;
}

// 非阻塞播放 需周期调用 时间基准 HAL_GetTick
void Music_Service(void)
{
    uint32_t now;

    if (s_playing == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_t0) < s_noteMs)
    {
        return; /* 当前音未到点 */
    }

    s_pos++;
    if (s_pos >= s_song->len)
    {
        Music_Stop(); /* 全曲播完 */
        return;
    }

    s_t0 = now;
    music_play_pos();
}

// 曲目数量只读
uint16_t Music_GetSongCount(void)
{
    return Music_SongListLen;
}

// 曲目编号查表 越界返回空指针
const Music_Song_t *Music_GetSong(uint16_t idx)
{
    if (idx >= Music_GetSongCount())
    {
        return NULL;
    }
    return &Music_SongList[idx];
}

#endif
