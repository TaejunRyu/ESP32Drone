#include "stdio.h"
#include <array>
#include <tuple>
struct aaa{
    int a;
    int b;
};

int func(float *arg1, float *arg2){
    arg1[0]= 10.0f;
    arg1[1]= 30.0f;
    arg1[2]= 50.0f;
return 1;
    

}

int  main(void) {
    float aaa[3]={1,2,3};
    float bbb[3]={4,5,6};

    int kk = func(aaa,bbb);

    printf("%f  %f   %f",aaa[0],aaa[1],aaa[2]);

    return 0;
}