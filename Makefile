all: decoder

decoder: pp.cpp data_buff.cpp
	g++ pp.cpp data_buff.cpp -o decoder

debug: pp.cpp data_buff.cpp
	g++ -g pp.cpp data_buff.cpp -o decoder

clean:
	rm -f decoder.exe
	