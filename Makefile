#
# Students' Makefile for the Malloc Lab
#
TEAM = JeongJongMun# 자신의 깃허브 이름으로 설정
VERSION = 1
# HANDINDIR = /afs/cs.cmu.edu/academic/class/15213-f01/malloclab/handin
HANDINDIR = Week05/

CC = gcc
CFLAGS = -Wall -O2 -m32 -w # 추가: -w = suppress warnings

OBJS = mdriver.o mm.o memlib.o fsecs.o fcyc.o clock.o ftimer.o

mdriver: $(OBJS)
	$(CC) $(CFLAGS) -o mdriver $(OBJS)

mdriver.o: mdriver.c fsecs.h fcyc.h clock.h memlib.h config.h mm.h
memlib.o: memlib.c memlib.h
mm.o: mm.c mm.h memlib.h
fsecs.o: fsecs.c fsecs.h config.h
fcyc.o: fcyc.c fcyc.h
ftimer.o: ftimer.c ftimer.h config.h
clock.o: clock.c clock.h

handin:
	mkdir -p $(HANDINDIR)/$(TEAM)
	cp -f mm.c $(HANDINDIR)/$(TEAM)/mm.c

handin-clean:
	rm -rf $(HANDINDIR)

clean:
	rm -f *~ *.o mdriver

# 사용법 (셸에서 명령어 입력)

# make
# = 컴파일

# ./mdriver
# = 실행

# make handin
# = mm.c 과제 복사 및 폴더 생성

# make handin-clean
# = handin 폴더 삭제

# make clean
# = 컴파일된 파일 삭제