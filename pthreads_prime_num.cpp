#include <stdlib.h>
//#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
vector<int> vInitialPrimes {};

struct args {
    int iStart = 0;
    int iEnd = 0;
    long long int iSumOfPrimes = 0;
};


void* primeNum(void* input)
{
    //printf("Thread[%ld] checking primes in %d and %d.\n",pthread_self(),((struct args*)input)->iStart,((struct args*)input)->iEnd);
    for (int i = ((struct args*)input)->iStart; i <= ((struct args*)input)->iEnd; i++) {
        //printf("Thread[%ld] checking whether %d is prime.\n",pthread_self(),i);
        int counter = 0;
       // printf("Thread[%ld] CheckPoint: A for Number %d.\n",pthread_self(),i);
        for(int prime : vInitialPrimes) {
            if(prime > floor(sqrt(i)))
                break;
            if(i%prime == 0) {
                //printf("Thread[%ld] CheckPoint: B for Number %d.\n",pthread_self(),i);
                counter++;
                break;
            }
        }

        if (counter == 0) {
          //  printf("Thread[%ld] CheckPoint: C for Number %d.\n",pthread_self(),i);
            vInitialPrimes.push_back(i);
            ((struct args*)input)->iSumOfPrimes += i;
           // printf("Thread[%ld] CheckPoint: D for Number %d.\n",pthread_self(),i);
           // printf("the prime number is %d\n", i);
        }
    }
    return input;
}

int main() {
    int iEnd = 1000000;
    int numWindows = 8;


    int initialStart = 2;
    int initialEnd = floor(sqrt(iEnd));

    struct args* initialPrimes = (struct args*)malloc(sizeof(struct args));
    initialPrimes->iStart = initialStart;
    initialPrimes->iEnd = initialEnd;
    initialPrimes->iSumOfPrimes = 0;

    primeNum((void*)initialPrimes);

    for(int i:vInitialPrimes)
        printf("Initial prime is:%d\n",i);

    printf("Sum is %lld\n",initialPrimes->iSumOfPrimes);

    pthread_t threads[numWindows];
    struct args* windowForPrimes[numWindows];

    int windowSize = ceil((iEnd - initialEnd+1)/numWindows);
    printf("windowSize is %d\n",(iEnd - initialEnd+1)/numWindows);

    int iStart = initialEnd+1;

    for(int i=0;i<numWindows ;i++) {
        windowForPrimes[i] = (struct args*)malloc(sizeof(struct args));
        windowForPrimes[i]->iStart = iStart;
        windowForPrimes[i]->iEnd = min(iStart+windowSize,iEnd);
        windowForPrimes[i]->iSumOfPrimes = 0;
        pthread_create(&threads[i], NULL, primeNum, (void*)windowForPrimes[i]);
        iStart = windowForPrimes[i]->iEnd + 1;
    }

    for(int i=0;i<numWindows ;i++)
        pthread_join(threads[i], NULL);

    long long int sumOfAllPrimes = initialPrimes->iSumOfPrimes;

    for(int i=0;i<numWindows ;i++) {
        printf("Main Thread[%ld] CheckPoint: A for Number %lld.\n",pthread_self(),windowForPrimes[i]->iSumOfPrimes);
        sumOfAllPrimes += windowForPrimes[i]->iSumOfPrimes;
    }


    printf("Sum is %lld\n",sumOfAllPrimes);

}
