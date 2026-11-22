/* Link-only target for Cortex-M7 size/callgraph validation; never flashed. */
#include "tinympc14.h"
int main(void)
{
    float x[TINYMPC14_NX]={0},f[TINYMPC14_NX]={0},ff[TINYMPC14_NU]={0};
    tinympc14_comp_t c;
    tinympc14_output_t out;
    Tinympc14_Init();
    Tinympc14_Output_Calc(.27f,.27f,0,0,0,0,&c);
    Tinympc14_Async_Input(x,x,f,ff,.27f,.27f);
    Tinympc14_Async_Solve();
    Tinympc14_Async_Step();
    Tinympc14_Output_Apply(ff,&c,ff,&out);
    Tinympc14_Async_Applied(out.u_applied);
    return 0;
}
