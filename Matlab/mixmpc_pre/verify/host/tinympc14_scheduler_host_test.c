#include "tinympc14_data.h"
#include <stdio.h>

#ifdef TINYMPC14_TERMINAL_CODEC_TEST
static int check_coordinates(const char *path)
{
    unsigned int shape[2], node, channel;
    short expected[TINYMPC14_COORD_RANK], decoded[TINYMPC14_COORD_RANK];
    FILE *input = fopen(path, "rb");
    if (!input) return 1;
    if (fread(shape, sizeof(shape), 1, input) != 1 ||
        shape[0] != TINYMPC14_COORD_PLANE_NODES *
                    (1U + sizeof(tinympc14_coord_plane_offsets) / sizeof(tinympc14_coord_plane_offsets[0])) ||
        shape[1] != TINYMPC14_COORD_RANK) {
        fclose(input);
        return 1;
    }
    for (node = 0; node < shape[0]; ++node) {
        if (fread(expected, sizeof(expected), 1, input) != 1) {
            fclose(input);
            return 1;
        }
        Tinympc14_Test_Decode_Node(node, decoded);
        for (channel = 0; channel < shape[1]; ++channel) {
            if (decoded[channel] != expected[channel]) {
                fprintf(stderr, "Coordinate mismatch at node %u channel %u: %d != %d\n",
                        node, channel, (int)decoded[channel], (int)expected[channel]);
                fclose(input);
                return 1;
            }
        }
    }
    if (fgetc(input) != EOF || ferror(input)) {
        fclose(input);
        return 1;
    }
    fclose(input);
    printf("PASS decoded %u nodes x %u coordinates, all int16 values exact\n", shape[0], shape[1]);
    return 0;
}
#endif

/* Stream the actual deployed scheduler matrices, independently of the solver. */
int main(int argc, char **argv)
{
    FILE *input, *output;
    float point[4];
    tinympc14_model_t model;
#ifdef TINYMPC14_TERMINAL_CODEC_TEST
    if (argc != 4 || check_coordinates(argv[3])) return 2;
#else
    if (argc != 3) return 2;
#endif
    input = fopen(argv[1], "rb");
    if (!input) return 3;
    output = fopen(argv[2], "wb");
    if (!output) { fclose(input); return 4; }
    while (fread(point, sizeof(float), 4, input) == 4) {
        Tinympc14_Lpv_Data_Update_Height(point[0], point[1], point[2], point[3], &model);
#define WRITE_FIELD(field) \
        if (fwrite(model.field, sizeof(model.field), 1, output) != 1) return 5
        WRITE_FIELD(Ad); WRITE_FIELD(Bd); WRITE_FIELD(fd); WRITE_FIELD(Kinf);
        WRITE_FIELD(Pinf); WRITE_FIELD(QuuInv); WRITE_FIELD(AmBKt);
#undef WRITE_FIELD
    }
    if (ferror(input) || fclose(output)) { fclose(input); return 6; }
    fclose(input);
    return 0;
}
