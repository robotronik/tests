all: a b
	mkfifo pipe_nomme
	0<pipe_nomme ./a | ./b 1>pipe_nomme

a: a.c
	gcc -o a a.c

b: b.c
	gcc -o b b.c

clean:
	rm a b pipe_nomme
