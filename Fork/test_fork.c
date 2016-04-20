#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>


#define EXEC "../../asservissement/build/PC/_DEBUG_/asser_robot"

#define READ   0
#define WRITE  1

int main(int argc, char const *argv[]) {
    int     to_child_stdin[2];
    pid_t   childpid;
    pipe(to_child_stdin);


    if((childpid = fork()) == -1) {
        perror("fork");
        exit(1);
    }



    if(childpid == 0) {
            /* Child process closes up input side of pipe */
            close(to_child_stdin[WRITE]);
            dup2(to_child_stdin[READ], STDIN_FILENO);

            execl("/bin/sh", "/bin/sh", "-c", EXEC, NULL);
            exit(0);

    } else {
        printf("pid : %d\n", childpid);
        /* Parent process closes up output side of pipe */
        close(to_child_stdin[READ]);
        write(to_child_stdin[WRITE], "x=1000\ny=1000\nxy_absolu()\n", 26);

        int stat;
        waitpid(childpid, &stat, 0);
        kill(childpid, SIGKILL);
    }

    return 0;
}
