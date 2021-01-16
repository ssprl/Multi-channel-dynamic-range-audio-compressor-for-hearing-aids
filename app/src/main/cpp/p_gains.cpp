//
// Created by yxh133130 on 9/11/2017.
//

#include "p_gains.h"
float *conv(float *a,float *b,int lena, int lenb)
{
    float *c = (float*)malloc((lena+lenb-1)*sizeof(float));
    int counter = 1;
    while (counter<lena+lenb)
    {
        if(counter<lenb)
        {
            for(int k=0;k<counter;k++)
            {
                c[counter-1] += a[counter-k-1]*b[k];
            }
        }
        else if(counter<=lena)
        {
            for(int k=0;k<lenb;k++)
            {
                c[counter-1] += a[counter-k-1]*b[k];
            }
        }
        else //tail part
        {
            for(int k=0;k<lena+lenb-counter;k++)
            {
                c[counter-1] += a[lena-k-1]*b[counter-lena+k];
            }
        }
        counter++;
    }
    return c;
}