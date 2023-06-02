all: decoder

decoder: pp.cpp data_buff.cpp
	g++ pp.cpp data_buff.cpp -o decoder -Wall

debug: pp.cpp data_buff.cpp
	g++ -g pp.cpp data_buff.cpp -o decoder -Wall

clean:
	rm -f decoder.exe
	