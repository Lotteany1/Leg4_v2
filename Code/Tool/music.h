/**
 * @file  music.h
 * @brief 蜂鸣器音符播放引擎头文件
 * @version  v1.0  2026-09-16
 */
#ifndef MUSIC_H
#define MUSIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* MIDI 音高编号 1 = C 调 低音组 1~7 = 48~59 中音组 = 60~71 中央 C = 60 高音组 = 72~83 倍高音 = 84~95
   升降半音直接 +/-1 本蜂鸣器按 4kHz 谐振配置 实际最高可发声约 B7 编号 107 */
#define MUSIC_NOTE_NUM 128U  /* MIDI 0~127，频率表长度 */
#define MUSIC_MAX_HZ   4000U /* Hz，超出蜂鸣器不再明显发声 视为休止 */

/* 半音，整体移调 实际音高 = 歌谱音高 + 此值
   蜂鸣器 4kHz 谐振 歌声基频 130~400Hz 直接播放低沉含糊 默认 +24 抬两个八度到约 520~2250Hz 近谐振点
   改 +12 为升一个八度 需保证 歌谱最高音 + 此值 <= 107 */
#define MUSIC_SHIFT_SEMIS 24U

    extern const uint16_t Music_NoteHz[MUSIC_NOTE_NUM];

#define MUSIC_NOTE_REST 0U

/* 歌谱每音两个字节 时值 + 音高 时值单位 1/32 音符 四分音符 = 8 单位 */
#define MUSIC_DUR_1  1U  /* 32 分音符 */
#define MUSIC_DUR_2  2U  /* 16 分音符 */
#define MUSIC_DUR_4  4U  /* 八分音符 */
#define MUSIC_DUR_8  8U  /* 四分音符 */
#define MUSIC_DUR_16 16U /* 二分音符 */
#define MUSIC_DUR_32 32U /* 全音符 4 拍 */
    /* 附点 = 原时值加一半 附点四分音符 = MUSIC_DUR_8 + MUSIC_DUR_4 = 12 */

    typedef struct
    {
        uint8_t dur;  /* 1/32 音符，时值 0 不使用 */
        uint8_t note; /* MIDI 编号 0~127，0 = 休止停声 */
    } Music_Note_t;

    typedef struct
    {
        const char *name;          /* 歌名，仅作注释与索引 */
        const Music_Note_t *notes; /* 歌谱数据，主旋律 */
        uint16_t len;              /* 音符个数 */
        uint16_t bpm;              /* 四分音符/分钟，速度 */
    } Music_Song_t;

    // MX_TIM12_Init 之后调一次 Music_Init 再周期调 Music_Service

    void Music_Init(void);        /* 定时器与状态初始化 */
    void Music_Tone(uint16_t hz); /* Hz，直接发声 0 或超范围静音 */

    void Music_Play(uint8_t idx);  /* 歌谱索引，越界即静默不播 */
    void Music_Stop(void);         /* 停声清播放标志 */
    uint8_t Music_IsPlaying(void); /* 1=播放中 */
    void Music_Service(void);      /* 周期调用 推进音符 */

    uint16_t Music_GetSongCount(void);               /* 内置歌谱条数 */
    const Music_Song_t *Music_GetSong(uint16_t idx); /* 越界返 NULL */

#ifdef __cplusplus
}
#endif

#endif /* MUSIC_H */
