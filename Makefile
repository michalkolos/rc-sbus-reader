all:

	gcc -Wall src/main.c -o bin/sbus_reader_run

clean:
	rm bin/sbus_reader_run
