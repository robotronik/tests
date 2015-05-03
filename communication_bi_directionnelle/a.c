#include <unistd.h>
#include <stdio.h>

int main(void)
{
    while(1) {
        sleep(1);
        char send = '+';
        printf("%c", send);
        fflush(stdout);
        fprintf(stderr, "a.c a envoyé %c\n", send);
        sleep(1);

        int receive;
        fprintf(stderr, "a.c est en attente d'un entier\n");
        scanf("%d", &receive);
        fprintf(stderr, "a.c a reçu %d\n", receive);
    }

    return 0;
}
