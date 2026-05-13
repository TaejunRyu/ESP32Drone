#include "stdio.h"
#include <array>
#include <tuple>

    

int  main(void) {
    float x[7] ={0.0f,0.0f,0.0f,0.0f,0.3f,0.5f,0.7f};

    float& q4 = x[4];
    float& q5 = x[5];
    float& q6 = x[6];

    q4 = 100.0f;

    printf("%f  %f   %f",q4,q5,q6);

    return 0;
}