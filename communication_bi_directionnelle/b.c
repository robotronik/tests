#include <unistd.h>
#include <stdio.h>

int main(void)
{
    while(1) {
        char receive;
        fprintf(stderr, "b.c est en attente d'un char\n");
        scanf("%c", &receive);
        fprintf(stderr, "b.c a reçu %c\n", receive);

        int send = 300;
        printf("%d\n", send);
        fflush(stdout);
        fprintf(stderr, "b.c a envoyé %d\n", send);
        sleep(1);
    }

    return 0;
}
