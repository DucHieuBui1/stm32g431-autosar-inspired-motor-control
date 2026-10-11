#include "EcuM.h"

int main(void)
{
    EcuM_Init();
    EcuM_Start();

    for (;;)
    {
        /* EcuM_Start does not return. */
    }
}
