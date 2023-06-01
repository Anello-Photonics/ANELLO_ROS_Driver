all: decoder

decoder: pp.cpp data_buff.cpp
	g++ pp.cpp data_buff.cpp -o decoder

clean:
	rm -f pp data_buff