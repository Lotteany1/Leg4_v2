/**
 * @file  songs.h
 * @brief 内置歌谱数据头文件
 * @version  v1.1  2026-10-09
 */
// 歌谱数据结构 Music_Note_t / Music_Song_t 与播放 API 见 music.h
// 加新歌 songs.c 新增 Music_Note_t 数组 Music_SongList[] 注册 Music_Play 索引播放
#ifndef SONGS_H
#define SONGS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "music.h"
#include <stdint.h>

    // 歌谱注册表 定义在 songs.c
    extern const Music_Song_t Music_SongList[]; /* 全部内置歌谱 */
    extern const uint16_t Music_SongListLen;    /* 内置歌谱数量 */

    // 歌曲索引枚举表 顺序与 Music_SongList[] 一致 按名选歌
    enum
    {
        SONG_IDX_GUANGYUYING = 0, /* 《光与影的对白》洛天依十周年官方生贺曲 */
        SONG_IDX_LANHUACAO,       /* 《兰花草》胡适词经典校园民歌 */
        SONG_IDX_KOIKOI,          /* 《恋ひ恋ふ縁》《千恋万花》OP */
        SONG_IDX_COUNT            /* 歌曲总数 勿手动改 与注册表同步 */
    };

#ifdef __cplusplus
}
#endif

#endif /* SONGS_H */
