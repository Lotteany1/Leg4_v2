/**
 * @file  songs.c
 * @brief 内置歌谱数据
 * @version  v1.3  2026-10-09
 */
#include "songs.h"

// 填谱方法 1=C 大调 中音1~7=MIDI60~71 高音1'~7'=72~83 低音1.~7.=48~59 升降半音=编号加减1 休止符={时值,0}
// 时值宏 1=32分 2=16分 4=八分 8=四分 16=二分 32=全音符 附点=原值+一半 4/4 一小节=32 个时值单位
// 例 |1 3 5 i| 高音两个八分+四分 -> {4,60},{4,64},{8,67},{8,72}
// 加新歌 新增 Music_Note_t 数组 Music_SongList[] 加注册行 songs.h 加 SONG_IDX_xxx

// 洛天依十周年官方生贺曲, 原谱 1=Eb 4/4, J=87; 本文件按 1=C 记谱录入
// 谱面后段标记 1=E(转调), 此处保持同一 1=C 关系, 原调听感可整体每音 +3 半音
// 结构: 主歌x2 - 副歌(||:反复, 含1.结尾) - 间奏 - 微弱旋律(前奏再现)
//       - 副歌回归 - 转调桥段(1=E) - 尾声, 共51小节(无前奏)
// 每行注释括号内为简谱记号(' = 高音, 附点已并入时值), 核对时直接对照
static const Music_Note_t s_song_guangying[] =
    {
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 79U},                /* 高音5 (5's) */
        {MUSIC_DUR_2, 79U},                /* 高音5 (5's) */
        {MUSIC_DUR_4, 79U},                /* 高音5 (5'e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_2, MUSIC_NOTE_REST},    /* 休止 十六分 (0.s) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 79U},                /* 高音5 (5's) */
        {MUSIC_DUR_2, 79U},                /* 高音5 (5's) */
        {MUSIC_DUR_4, 79U},                /* 高音5 (5'e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 79U},                /* 高音5 (5'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 57U},  /* 低音6 (6.e.) */
        {MUSIC_DUR_2, 76U},                /* 高音3 (3's) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 59U},  /* 低音7 (7.e.) */
        {MUSIC_DUR_2, 74U},                /* 高音2 (2's) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_8, 72U},                /* 高音1 (1'q) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_8, 57U},                /* 低音6 (6.q) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_32, 57U},               /* 低音6 (6.w) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},    /* 休止 八分 (0.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_16, 48U},               /* 低音1 (1.h) */
        {MUSIC_DUR_8, 48U},                /* 低音1 (1.q) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 48U},                /* 低音1 (1.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 53U},                /* 低音4 (4.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_8, MUSIC_NOTE_REST},    /* 休止 四分 (0.q) */
        {MUSIC_DUR_8, MUSIC_NOTE_REST},    /* 休止 四分 (0.q) */
        {MUSIC_DUR_8, MUSIC_NOTE_REST},    /* 休止 四分 (0.q) */
        {MUSIC_DUR_8, MUSIC_NOTE_REST},    /* 休止 四分 (0.q) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 81U},                /* 高音6 (6'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_8, 57U},                /* 低音6 (6.q) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_16, 76U},               /* 高音3 (3'h) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_2, 74U},                /* 高音2 (2's) */
        {MUSIC_DUR_2, 76U},                /* 高音3 (3's) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_2, 55U},                /* 低音5 (5.s) */
        {MUSIC_DUR_2, 57U},                /* 低音6 (6.s) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 76U},                /* 高音3 (3'e) */
        {MUSIC_DUR_2, 74U},                /* 高音2 (2's) */
        {MUSIC_DUR_2, 72U},                /* 高音1 (1's) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 57U},  /* 低音6 (6.e.) */
        {MUSIC_DUR_4, 57U},                /* 低音6 (6.e) */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 59U},  /* 低音7 (7.e.) */
        {MUSIC_DUR_4, 59U},                /* 低音7 (7.e) */
        {MUSIC_DUR_8, 59U},                /* 低音7 (7.q) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_8, 74U},                /* 高音2 (2'q) */
        {MUSIC_DUR_8, 76U},                /* 高音3 (3'q) */
        {MUSIC_DUR_4, 74U},                /* 高音2 (2'e) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_8, 57U},                /* 低音6 (6.q) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_8, 48U},                /* 低音1 (1.q) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_16 + MUSIC_DUR_8, 72U}, /* 高音1 (1'h.) */
        {MUSIC_DUR_4, 72U},                /* 高音1 (1'e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_8, 57U},                /* 低音6 (6.q) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 55U},                /* 低音5 (5.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_4, 52U},                /* 低音3 (3.e) */
        {MUSIC_DUR_16, 48U},               /* 低音1 (1.h) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_8, 55U},                /* 低音5 (5.q) */
        {MUSIC_DUR_2, 52U},                /* 低音3 (3.s) */
        {MUSIC_DUR_2, 50U},                /* 低音2 (2.s) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 48U},                /* 低音1 (1.e) */
        {MUSIC_DUR_4, 50U},                /* 低音2 (2.e) */
        {MUSIC_DUR_32, 48U},               /* 低音1 (1.w) */
};

// 胡适词, 陈贤德/张弼曲; 1=C 2/4, ♩≈96; 台湾校园歌曲, 依演唱版记谱(无前奏)
// 结构: A B C D A B C D, 共8乐句; 括号内为简谱记号(6. = 低音6, 7. = 低音7)
// A: |6 3 3 3|3. 2|1. 2 1 7|6 -|   B: |6 6 6 6|6. 5|3 5 5 4|3 -|
// C: |3 6 6 5|3. 2|1. 2 1 7|6 3|   D: |3 1 1 7|6. 3|2 1 7 5|6 -|
static const Music_Note_t s_song_lanhuacao[] =
    {
        // 第1乐句 A: 我从山中来, 带着兰花草
        {MUSIC_DUR_4, 57U},               /* 我 6. 八分 */
        {MUSIC_DUR_4, 64U},               /* 从 3  八分 */
        {MUSIC_DUR_4, 64U},               /* 山 3  八分 */
        {MUSIC_DUR_4, 64U},               /* 中 3  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 64U}, /* 来 3. 附点四分 */
        {MUSIC_DUR_4, 62U},               /*    2  八分 */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 60U}, /* 带 1. 附点八分 */
        {MUSIC_DUR_2, 62U},               /* 着 2  十六分 */
        {MUSIC_DUR_4, 60U},               /* 兰 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 花 7. 八分 */
        {MUSIC_DUR_16, 57U},              /* 草 6. 二分 */

        // 第2乐句 B: 种在小园中, 希望花开早
        {MUSIC_DUR_4, 69U},               /* 种 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 在 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 小 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 园 6  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 69U}, /* 中 6. 附点四分 */
        {MUSIC_DUR_4, 67U},               /*    5  八分 */
        {MUSIC_DUR_4, 64U},               /* 希 3  八分 */
        {MUSIC_DUR_4, 67U},               /* 望 5  八分 */
        {MUSIC_DUR_4, 67U},               /* 花 5  八分 */
        {MUSIC_DUR_4, 65U},               /* 开 4  八分 */
        {MUSIC_DUR_16, 64U},              /* 早 3  二分 */

        // 第3乐句 C: 一日看三回, 看得花时过
        {MUSIC_DUR_4, 64U},               /* 一 3  八分 */
        {MUSIC_DUR_4, 69U},               /* 日 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 看 6  八分 */
        {MUSIC_DUR_4, 67U},               /* 三 5  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 64U}, /* 回 3. 附点四分 */
        {MUSIC_DUR_4, 62U},               /*    2  八分 */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 60U}, /* 看 1. 附点八分 */
        {MUSIC_DUR_2, 62U},               /* 得 2  十六分 */
        {MUSIC_DUR_4, 60U},               /* 花 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 时 7. 八分 */
        {MUSIC_DUR_8, 57U},               /* 过 6. 四分 */
        {MUSIC_DUR_8, 52U},               /*    3. 四分 */

        // 第4乐句 D: 兰花却依然, 苞也无一个
        {MUSIC_DUR_4, 52U},               /* 兰 3. 八分 */
        {MUSIC_DUR_4, 60U},               /* 花 1  八分 */
        {MUSIC_DUR_4, 60U},               /* 却 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 依 7. 八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 57U}, /* 然 6. 附点四分 */
        {MUSIC_DUR_4, 52U},               /*    3. 八分 */
        {MUSIC_DUR_4, 62U},               /* 苞 2  八分 */
        {MUSIC_DUR_4, 60U},               /* 也 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 无 7. 八分 */
        {MUSIC_DUR_4, 55U},               /* 一 5. 八分 */
        {MUSIC_DUR_16, 57U},              /* 个 6. 二分 */

        // 第5乐句 A: 转眼秋天到, 移兰入暖房
        {MUSIC_DUR_4, 57U},               /* 转 6. 八分 */
        {MUSIC_DUR_4, 64U},               /* 眼 3  八分 */
        {MUSIC_DUR_4, 64U},               /* 秋 3  八分 */
        {MUSIC_DUR_4, 64U},               /* 天 3  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 64U}, /* 到 3. 附点四分 */
        {MUSIC_DUR_4, 62U},               /*    2  八分 */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 60U}, /* 移 1. 附点八分 */
        {MUSIC_DUR_2, 62U},               /* 兰 2  十六分 */
        {MUSIC_DUR_4, 60U},               /* 入 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 暖 7. 八分 */
        {MUSIC_DUR_16, 57U},              /* 房 6. 二分 */

        // 第6乐句 B: 朝朝频顾惜, 夜夜不相忘
        {MUSIC_DUR_4, 69U},               /* 朝 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 朝 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 频 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 顾 6  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 69U}, /* 惜 6. 附点四分 */
        {MUSIC_DUR_4, 67U},               /*    5  八分 */
        {MUSIC_DUR_4, 64U},               /* 夜 3  八分 */
        {MUSIC_DUR_4, 67U},               /* 夜 5  八分 */
        {MUSIC_DUR_4, 67U},               /* 不 5  八分 */
        {MUSIC_DUR_4, 65U},               /* 相 4  八分 */
        {MUSIC_DUR_16, 64U},              /* 忘 3  二分 */

        // 第7乐句 C: 期待春花开, 能将夙愿偿
        {MUSIC_DUR_4, 64U},               /* 期 3  八分 */
        {MUSIC_DUR_4, 69U},               /* 待 6  八分 */
        {MUSIC_DUR_4, 69U},               /* 春 6  八分 */
        {MUSIC_DUR_4, 67U},               /* 花 5  八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 64U}, /* 开 3. 附点四分 */
        {MUSIC_DUR_4, 62U},               /*    2  八分 */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 60U}, /* 能 1. 附点八分 */
        {MUSIC_DUR_2, 62U},               /* 将 2  十六分 */
        {MUSIC_DUR_4, 60U},               /* 夙 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 愿 7. 八分 */
        {MUSIC_DUR_8, 57U},               /* 偿 6. 四分 */
        {MUSIC_DUR_8, 52U},               /*    3. 四分 */

        // 第8乐句 D: 满庭花簇簇, 添得许多香
        {MUSIC_DUR_4, 52U},               /* 满 3. 八分 */
        {MUSIC_DUR_4, 60U},               /* 庭 1  八分 */
        {MUSIC_DUR_4, 60U},               /* 花 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 簇 7. 八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 57U}, /* 簇 6. 附点四分 */
        {MUSIC_DUR_4, 52U},               /*    3. 八分 */
        {MUSIC_DUR_4, 62U},               /* 添 2  八分 */
        {MUSIC_DUR_4, 60U},               /* 得 1  八分 */
        {MUSIC_DUR_4, 59U},               /* 许 7. 八分 */
        {MUSIC_DUR_4, 55U},               /* 多 5. 八分 */
        {MUSIC_DUR_16, 57U},              /* 香 6. 二分 */
};

// 柚子社《千恋万花》OP 主旋律, 原谱 1=E 4/4; 按 1=E 简谱录入(1=E 即中音1=64)
// 谱面音域 低音4~中音7, 已整体降一个八度录入, 配合 MUSIC_SHIFT_SEMIS(+24) 抬回原调
static const Music_Note_t s_song_koikoi[] =
    {
        // 前奏
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_4, 61U},             /* 低音6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 61U},             /* 低音6 四分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_2, 61U},             /* 低音6 十六分 */
        {MUSIC_DUR_2, 71U},             /* 5 十六分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_2, 66U},             /* 2 十六分 */
        {MUSIC_DUR_2, 66U},             /* 2 十六分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 61U},             /* 低音6 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_2, 64U},             /* 1 十六分 */
        {MUSIC_DUR_2, 64U},             /* 1 十六分 */
        {MUSIC_DUR_4, 61U},             /* 低音6 八分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_8, 59U},             /* 低音5 四分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 69U},             /* 4 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 66U},             /* 2 四分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_8, 59U},             /* 低音5 四分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 63U},             /* 低音7 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 69U},             /* 4 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 65U},             /* #1 八分 */
        {MUSIC_DUR_8, 65U},             /* #1 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */

        // 第1小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 66U},             /* 2 四分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */

        // 第2小节
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 69U}, /* 4 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */

        // 第3小节
        {MUSIC_DUR_8, 59U},             /* 低音5 四分 */
        {MUSIC_DUR_4, 57U},             /* 低音4 八分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */
        {MUSIC_DUR_8, 59U},             /* 低音5 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */

        // 第4小节
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */

        // 第5小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 66U},             /* 2 四分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */

        // 第6小节
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 69U}, /* 4 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 69U}, /* 4 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */

        // 第7小节
        {MUSIC_DUR_8, 71U},               /* 5 四分 */
        {MUSIC_DUR_4, 64U},               /* 1 八分 */
        {MUSIC_DUR_4, 64U},               /* 1 八分 */
        {MUSIC_DUR_8 + MUSIC_DUR_4, 64U}, /* 1 附点四分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST},   /* 休止 八分 */

        // 第8小节
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */

        // 第9小节
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_4, 69U}, /* 4 八分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_8, 69U}, /* 4 四分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */

        // 第10小节
        {MUSIC_DUR_4, 68U},  /* 3 八分 */
        {MUSIC_DUR_8, 66U},  /* 2 四分 */
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_16, 66U}, /* 2 二分 */

        // 第11小节
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */

        // 第12小节
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_8, 64U}, /* 1 四分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_8, 64U}, /* 1 四分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_4, 75U}, /* 7 八分 */

        // 第13小节
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_2, 68U},             /* 3 十六分 */
        {MUSIC_DUR_2, MUSIC_NOTE_REST}, /* 休止 十六分 */
        {MUSIC_DUR_4, 69U},             /* 4 八分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */

        // 第14小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */

        // 第15小节
        {MUSIC_DUR_32, 66U}, /* 2 全音符 */

        // 第16小节
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */

        // 第17小节
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */

        // 第18小节
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 59U},             /* 低音5 四分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 59U},             /* 低音5 八分 */

        // 第19小节
        {MUSIC_DUR_8, 73U}, /* 6 四分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */

        // 第20小节
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 64U},             /* 1 八分 */
        {MUSIC_DUR_8, 66U},             /* 2 四分 */
        {MUSIC_DUR_8, 68U},             /* 3 四分 */
        {MUSIC_DUR_8, MUSIC_NOTE_REST}, /* 休止 四分 */

        // 第21小节
        {MUSIC_DUR_8, 73U}, /* 6 四分 */
        {MUSIC_DUR_4, 71U}, /* 5 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_4, 71U}, /* 5 八分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */

        // 第22小节
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */
        {MUSIC_DUR_8, 73U}, /* 6 四分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_4, 75U}, /* 7 八分 */

        // 第23小节
        {MUSIC_DUR_8, 64U}, /* 1 四分 */
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_8, 71U}, /* 5 四分 */

        // 第24小节
        {MUSIC_DUR_32, 68U}, /* 3 全音符 */

        // 第25小节
        {MUSIC_DUR_16, 68U},            /* 3 二分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */

        // 第26小节
        {MUSIC_DUR_8, 73U},  /* 6 四分 */
        {MUSIC_DUR_4, 71U},  /* 5 八分 */
        {MUSIC_DUR_4, 73U},  /* 6 八分 */
        {MUSIC_DUR_16, 68U}, /* 3 二分 */

        // 第27小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_8, 68U},             /* 3 四分 */

        // 第28小节
        {MUSIC_DUR_4 + MUSIC_DUR_2, 66U}, /* 2 附点八分 */
        {MUSIC_DUR_2, 66U},               /* 2 十六分 */
        {MUSIC_DUR_4, 68U},               /* 3 八分 */
        {MUSIC_DUR_4, 71U},               /* 5 八分 */
        {MUSIC_DUR_4 + MUSIC_DUR_2, 73U}, /* 6 附点八分 */
        {MUSIC_DUR_2, 71U},               /* 5 十六分 */
        {MUSIC_DUR_8, 68U},               /* 3 四分 */

        // 第29小节
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_4, 68U},  /* 3 八分 */
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_4, 64U},  /* 1 八分 */
        {MUSIC_DUR_16, 68U}, /* 3 二分 */

        // 第30小节
        {MUSIC_DUR_8, 66U},  /* 2 四分 */
        {MUSIC_DUR_4, 64U},  /* 1 八分 */
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_16, 73U}, /* 6 二分 */

        // 第31小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */
        {MUSIC_DUR_8, 64U},             /* 1 四分 */
        {MUSIC_DUR_4, 66U},             /* 2 八分 */

        // 第32小节
        {MUSIC_DUR_32, 68U}, /* 3 全音符 */

        // 第33小节
        {MUSIC_DUR_16, 68U},            /* 3 二分 */
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 68U},             /* 3 八分 */
        {MUSIC_DUR_8, 71U},             /* 5 四分 */

        // 第34小节
        {MUSIC_DUR_8, 73U},  /* 6 四分 */
        {MUSIC_DUR_4, 71U},  /* 5 八分 */
        {MUSIC_DUR_4, 73U},  /* 6 八分 */
        {MUSIC_DUR_16, 68U}, /* 3 二分 */

        // 第35小节
        {MUSIC_DUR_4, MUSIC_NOTE_REST}, /* 休止 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_4, 73U},             /* 6 八分 */
        {MUSIC_DUR_4, 71U},             /* 5 八分 */
        {MUSIC_DUR_8, 68U},             /* 3 四分 */

        // 第36小节
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */
        {MUSIC_DUR_4, 71U}, /* 5 八分 */
        {MUSIC_DUR_8, 73U}, /* 6 四分 */
        {MUSIC_DUR_8, 68U}, /* 3 四分 */

        // 第37小节
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_4, 64U},  /* 1 八分 */
        {MUSIC_DUR_4, 66U},  /* 2 八分 */
        {MUSIC_DUR_4, 68U},  /* 3 八分 */
        {MUSIC_DUR_16, 73U}, /* 6 二分 */

        // 第38小节
        {MUSIC_DUR_8, 73U}, /* 6 四分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 73U}, /* 6 八分 */
        {MUSIC_DUR_8, 66U}, /* 2 四分 */
        {MUSIC_DUR_4, 64U}, /* 1 八分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */

        // 第39小节
        {MUSIC_DUR_8, 68U}, /* 3 四分 */
        {MUSIC_DUR_4, 66U}, /* 2 八分 */
        {MUSIC_DUR_4, 68U}, /* 3 八分 */
        {MUSIC_DUR_8, 71U}, /* 5 四分 */
        {MUSIC_DUR_8, 75U}, /* 7 四分 */

        // 第40小节
        {MUSIC_DUR_32, 75U}, /* 7 全音符 */

        // 第41小节
        {MUSIC_DUR_32, 73U}, /* 6 全音符 */
};

// 歌谱注册表
// 需要加歌 上面新增数组 这里加一行 songs.h 加索引枚举
const Music_Song_t Music_SongList[] =
    {
        {
            "LuoTianYi_GuangYuYingDeDuiBai",
            s_song_guangying,
            (uint16_t)(sizeof(s_song_guangying) / sizeof(s_song_guangying[0])),
            72U, /* 播放速度: 四分音符/分钟; 原谱标 J=87 实测偏快, 降到 72 听感合适 */
        },
        {
            "LanHuaCao",
            s_song_lanhuacao,
            (uint16_t)(sizeof(s_song_lanhuacao) / sizeof(s_song_lanhuacao[0])),
            96U, /* 《兰花草》1=C 2/4, 原速约 96 */
        },
        {
            "KoiKoiEnchina",
            s_song_koikoi,
            (uint16_t)(sizeof(s_song_koikoi) / sizeof(s_song_koikoi[0])),
            144U, /* 《恋ひ恋ふ縁》1=E 4/4, 源谱四分音符约 417ms 合 J=144 */
        },
};

const uint16_t Music_SongListLen =
    (uint16_t)(sizeof(Music_SongList) / sizeof(Music_SongList[0]));
