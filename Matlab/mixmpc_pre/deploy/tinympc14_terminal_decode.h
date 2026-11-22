/* Embedded in the scheduler translation unit after tinympc14_data.h.
 * The generated grid bounds restrict node to [0, nAngle * planeNodes).
 * The encoder checks each prefix width (0..17), each stride and the last
 * byte of every plane. A trailing zero word makes the final preload valid. */
static void tinympc14_terminal_decode_node(unsigned int node, short *out)
{
    unsigned int plane = node / TINYMPC14_COORD_PLANE_NODES;
    unsigned int local_node = node % TINYMPC14_COORD_PLANE_NODES;
    const short *anchor = &tinympc14_coord_anchor[local_node * TINYMPC14_COORD_RANK];
    unsigned int c;
    if (plane == TINYMPC14_COORD_ANCHOR_PLANE) {
        for (c = 0; c < TINYMPC14_COORD_RANK; ++c) {
            out[c] = anchor[c];
        }
    } else {
        unsigned int index = plane - (plane > TINYMPC14_COORD_ANCHOR_PLANE);
        const unsigned short *prefix = &tinympc14_coord_bit_offsets[index * (TINYMPC14_COORD_RANK + 1U)];
        unsigned int stride = (prefix[TINYMPC14_COORD_RANK] + 7U) / 8U;
        unsigned int offset = tinympc14_coord_plane_offsets[index] + local_node * stride;
        const unsigned int *src = tinympc14_coord_payload + offset / 4U;
        unsigned int bit = (offset % 4U) * 8U;
        unsigned int word = *src++;
        for (c = 0; c < TINYMPC14_COORD_RANK; ++c) {
            unsigned int width = prefix[c + 1U] - prefix[c];
            unsigned int next = bit + width;
            unsigned int zigzag = word >> bit;
            int delta;
            if (next >= 32U) {
                word = *src++;
                if (next > 32U) {
                    zigzag |= word << (32U - bit);
                }
                next -= 32U;
            }
            bit = next;
            zigzag &= (1U << width) - 1U;
            delta = (int)(zigzag >> 1);
            if (zigzag & 1U) {
                delta = -1 - delta;
            }
            out[c] = (short)((int)anchor[c] + delta);
        }
    }
}

#ifdef TINYMPC14_TERMINAL_CODEC_TEST
void Tinympc14_Test_Decode_Node(unsigned int node, short *out)
{
    tinympc14_terminal_decode_node(node, out);
}
#endif
