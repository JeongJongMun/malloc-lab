/*
 * < mm-segregated-buddy.c >
 * 이 방법은 분리 가용 리스트(Segregated Free List)를 사용하여 메모리 할당을 관리하는 malloc 패키지를 구현하였다.
 * 다수의 명시적 가용 리스트를 유지하며, 각 리스트는 2^k의 크기 클래스로 구분된다.
 * 각 클래스는 2^k-1 초과 2^k 이하의 크기를 가진 할당 요청을 2^k 블록을 저장하고, Buddy System 방식을 사용한다.
 * 이 방법에서는 더블 워드 정렬을 사용하며, 모든 블록의 헤더에 크기와 할당 비트를 저장한다. (1 Word = 4 Byte)
 * 최소 블록 크기는 4 Word (16 Byte)이며, 헤더는 1 Word이다.
 *
 */

/*
 * < 분리 가용 리스트(Segregated Free List) - Buddy System >
 *
 *                                                     heap_listp
 *                                                         |
 *                                                         |
 *                                                         |
 *                                                         V
 * ---------------------------------------------------------
 * | Alignment Padding | Prologue Header | Prologue Footer | . . . . 아래에 이어서 그리겠음
 * ---------------------------------------------------------
 *                                                                                                                            mem_brk
 *                                                                                                                               |
 *                                                                                                                               |
 *                                                                                                                               |
 * SEGREGATED_LIST_SIZE 만큼의 Segregate Free List Root 생성. 각각의 루트가 크기 클래스임.                                          V
 * -------------------------------------------------------------------------------------------------------------------------------
 * Segregate Free List Root | ... | Segregate Free List Root | Free Block | ... | Free Block | ...| Free Block | Epilogue Header |
 * -------------------------------------------------------------------------------------------------------------------------------
 *
 * Alignment Padding: 8의 배수로 맞추기 위한 패딩 (값 = 0)
 * Prologue Header: 가용 블록의 시작을 나타내는 헤더 (값 = 8, 할당 비트 = 1)
 * Prologue Footer: 가용 블록의 끝을 나타내는 풋터 (값 = 8, 할당 비트 = 1)
 * Epilogue Header: 힙의 끝을 나타내는 헤더 (값 = 0, 할당 비트 = 1)
 *
 *
 * < 할당 블록 구조 >
 * 31 . . . . . . . . . . . . . . . . . .  0            alloc bit = 001 : 할당 상태
 * -----------------------------------------            alloc bit = 000 : 가용 상태
 * | Block size                | alloc bit | Header
 * -----------------------------------------
 * |                                       |
 * |                                       |
 * |                Payload                |
 * |                                       |
 * |                                       |
 * -----------------------------------------
 * |           Padding(Optional)           |
 * -----------------------------------------
 *
 *
 * < 가용 블록 구조 >
 * 31 . . . . . . . . . . . . . . . . . .  0            alloc bit = 001 : 할당 상태
 * -----------------------------------------            alloc bit = 000 : 가용 상태
 * | Block size                | alloc bit | Header
 * -----------------------------------------
 * |           Predecessor Pointer         |    }
 * -----------------------------------------    }
 * |            Successor Pointer          |    } Old Payload
 * -----------------------------------------    }
 * |                                       |    }
 * |                                       |    }
 * -----------------------------------------
 * |           Padding(Optional)           |
 * -----------------------------------------
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include "mm.h"
#include "memlib.h"

#define FIRST_FIT // 39 + 40 = 79

// 단일 워드 (4) 또는 더블 워드 (8) 정렬
#define ALIGNMENT 8

// size를 8의 배수로 올림
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~0x7)

// 기본 상수와 매크로들
#define WSIZE 4             // Word와 헤더와 풋터의 크기 4Byte
#define DSIZE 8             // 더블 워드의 크기 8Byte
#define CHUNKSIZE (1 << 12) // 초기 가용 블록과 힙 확장을 위한 기본 크기 4096Byte

#define MAX(x, y) ((x) > (y) ? (x) : (y))

// 크기와 할당 비트를 통합해서 헤더와 풋터에 저장할 수 있는 워드를 반환
#define PACK(size, alloc) ((size) | (alloc))

// 주소 p에서 워드를 읽기 (p는 void *)
#define GET(p) (*(unsigned int *)(p))
// 주소 p에 val을 쓰기
#define PUT(p, val) (*(unsigned int *)(p) = (val))

// 주소 p에서 크기를 읽기
#define GET_SIZE(p) (GET(p) & ~0x7)
// 주소 p에서 할당 비트를 읽기
#define GET_ALLOC(p) (GET(p) & 0x1)

// 블록 포인터의 헤더 주소를 반환
#define HDRP(bp) ((char *)(bp)-WSIZE)

// 다음 블록 포인터를 반환
#define NEXT_BLKP(bp) ((char *)(bp) + GET_SIZE(((char *)(bp)-WSIZE)))
// 이전 블록 포인터를 반환
#define PREV_BLKP(bp) ((char *)(bp)-GET_SIZE(((char *)(bp)-DSIZE)))

/* Explicit Free List를 위한 매크로 */
// 이전 가용 블록의 포인터를 반환
#define GET_PRED(bp) (*(void **)(bp))
// 다음 가용 블록의 포인터를 반환
#define GET_SUCC(bp) (*(void **)(bp + WSIZE))

/* Segregated Free List를 위한 매크로 */
// 가용 리스트의 개수 2^20까지
#define SEGREGATED_LIST_SIZE 20
// 해당 가용 리스트의 루트
#define GET_ROOT(index) (*(void **)(heap_listp + (index * WSIZE)))

team_t team = {
    "호둘치",
    "정종문",
    "whdans4005@gmail.com",
    "백강민",
    "qwey1584@gmail.com",
    "연선애",
    "ifindary@gmail.com",
};

/* 전역 변수 */
static void *heap_listp; // 가용 블록 리스트의 처음 블록을 가리키는 포인터

/* 함수 프로토타입 */
static void *coalesce(void *bp);
static void *extend_heap(size_t words);
static void *find_fit(size_t asize);
static void place(void *bp, size_t asize);
static void remove_free_block(void *bp);
static void add_free_block(void *bp);
int get_class(size_t size);

/*
 * mm_init - malloc 패키지 초기화. Prologue block과 Epilogue block을 생성하고 초기 힙을 구성한다.
 * 분리 가용 리스트에서는 각 클래스별로 가용 리스트를 생성한다.
 */
int mm_init(void)
{
    // 8워드 크기의 초기 힙 생성
    if ((heap_listp = mem_sbrk((SEGREGATED_LIST_SIZE + 4) * WSIZE)) == (void *)-1)
        return -1;
    PUT(heap_listp, 0);                                                                                  // Alignment padding
    PUT(heap_listp + (1 * WSIZE), PACK((SEGREGATED_LIST_SIZE + 2) * WSIZE, 1));                          // Prologue header (Header + Footer + Segregated Free List)
    for (int i = 0; i < SEGREGATED_LIST_SIZE; i++)                                                       // Segragated Free List 초기화
        PUT(heap_listp + ((2 + i) * WSIZE), NULL);                                                       // 각각의 Segregated Free List Root
    PUT(heap_listp + ((SEGREGATED_LIST_SIZE + 2) * WSIZE), PACK((SEGREGATED_LIST_SIZE + 2) * WSIZE, 1)); // Prologue footer (Header + Footer + Segregated Free List)
    PUT(heap_listp + ((SEGREGATED_LIST_SIZE + 3) * WSIZE), PACK(0, 1));                                  // Epilogue header

    heap_listp += (2 * WSIZE);                  // 첫 번째 분리 가용 블록의 시작 주소
    if (extend_heap(CHUNKSIZE / WSIZE) == NULL) // CHUNKSIZE만큼의 가용 블록으로 초기 힙을 확장한다.
        return -1;

    return 0;
}

/* extend_heap - 힙을 확장하고 새로운 가용 블록을 생성한다.
 * 초기화 시에 매개변수 words로 1024이 들어온다면, size는 1024 * 4 = 4096이 된다.
 * 4096은 16진수로 0x1000이다. 따라서 program break의 주소는 0x1000만큼 증가한다.
 */
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size;

    // 가용 블록의 크기는 8의 배수로 올림
    size = (words % 2 == 1) ? (words + 1) * WSIZE : words * WSIZE;
    if ((long)(bp = mem_sbrk(size)) == -1)
        return NULL;

    PUT(HDRP(bp), PACK(size, 0));         // 새로운 가용 블록 헤더
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1)); // 새로운 에필로그 헤더
    return coalesce(bp);                  // 이전 블록이 가용 상태이면 통합
}

/*
 * mm_malloc - 블럭을 할당하고 적합한 블록을 찾지 못했을 때 힙을 확장한다.
 * 블록의 크기는 최소 16바이트 크기(4 워드)의 블록으로 구성
 * 12바이트는 정렬 조건을 만족하기 위해, 4바이트는 헤더를 위해 사용
 */
void *mm_malloc(size_t size) // size는 헤더를 제외한 블록의 크기
{
    size_t asize = 16; // 헤더를 포함한 조정된 블록의 크기
    size_t extendsize; // 적합한 블록을 찾지 못했을 때 힙을 확장하는 양
    char *bp;

    // 가짜 블록을 할당하지 않음
    if (size == 0)
        return NULL;

    // 최소 블록 크기보다 작으면 2배씩 증가
    while (asize < size + DSIZE)
        asize <<= 1;

    // 가용 리스트에서 적합한 블록을 찾음
    if ((bp = find_fit(asize)) != NULL)
    {
        place(bp, asize);
        return bp;
    }

    // 적합한 블록을 찾지 못했을 때 힙을 확장하고 새로운 블록을 할당
    extendsize = MAX(asize, CHUNKSIZE);
    if ((bp = extend_heap(extendsize / WSIZE)) == NULL)
        return NULL;
    place(bp, asize);

    return bp;
}

/*
 * mm_free - 블록을 가용 상태로 설정하고 인접 가용 블록과 통합한다.
 */
void mm_free(void *bp)
{
    size_t size = GET_SIZE(HDRP(bp));
    PUT(HDRP(bp), PACK(size, 0));
    coalesce(bp);
}

/*
 * place - 가용 블록의 시작 부분에 asize 바이트의 블록을 배치하고 나머지가 최소 블록 크기 이상이면 분할한다.
 */
static void place(void *bp, size_t allocate_size) // allocate_size는 헤더와 풋터를 포함한 블록의 크기 (2^k 로 맞춰져서 들어옴)
{
    remove_free_block(bp); // 명시적 가용 리스트에서 가용 블록을 제거
    size_t chunk_size = GET_SIZE(HDRP(bp));

    while (allocate_size != chunk_size)
    {
        chunk_size >>= 1;                                // 블록을 반으로 분할
        PUT(HDRP(bp + chunk_size), PACK(chunk_size, 0)); // 버디를 가용 블록으로 변경
        add_free_block(bp + chunk_size);                 // 버디를 가용 블록 리스트에 추가
    }
    PUT(HDRP(bp), PACK(chunk_size, 1)); // 가용 블록을 할당 상태로 변경
}

/*
 * remove_free_block - 명시적 가용 리스트에서 가용 블록을 제거한다.
 */
static void remove_free_block(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));

    if (bp == GET_ROOT(class))
    {
        GET_ROOT(class) = GET_SUCC(GET_ROOT(class));
        return;
    }
    GET_SUCC(GET_PRED(bp)) = GET_SUCC(bp); // 이전 블록의 다음 블록을 변경
    if (GET_SUCC(bp) != NULL)              // 다음 블록의 이전 블록을 변경
        GET_PRED(GET_SUCC(bp)) = GET_PRED(bp);
}

/*
 * add_free_block - 명시적 가용 리스트에 가용 블록을 추가한다.
 */
static void add_free_block(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));

    GET_SUCC(bp) = GET_ROOT(class); // 새로운 블록의 다음 블록을 가용 리스트의 시작 블록으로 설정
    if (GET_ROOT(class) != NULL)
        GET_PRED(GET_ROOT(class)) = bp; // 가용 리스트 시작 블록의 이전 블록을 새로운 블록으로 설정
    GET_ROOT(class) = bp;               // 가용 리스트 시작 블록을 새로운 블록으로 설정
}

/*
 * get_class - 적합한 가용 리스트를 찾기 위한 인덱스를 반환한다.
 */
int get_class(size_t size)
{
    int next_power_of_2 = 1;
    int class = 0;

    while (next_power_of_2 < size && class + 1 < SEGREGATED_LIST_SIZE)
    {
        next_power_of_2 <<= 1;
        class ++;
    }
    return class;
}

/*
 * coalesce - 경계 태그 합치기. 합쳐진 블록의 포인터를 반환한다.
 */
static void *coalesce(void *bp)
{
    add_free_block(bp);
    size_t csize = GET_SIZE(HDRP(bp)); // 현재 블록의 크기
    void *root = heap_listp + (SEGREGATED_LIST_SIZE + 1) * WSIZE;
    void *left_buddyp;
    void *right_buddyp;

    while (1)
    {
        // 좌우 버디의 bp 파악
        if ((bp - root) & csize) // 현재 블록에서 힙까지의 메모리 합(bp - root)과 csize가 중복되는 비트가 있다면 현재 블록은 오른쪽 버디에 해당
        {
            left_buddyp = bp - csize;
            right_buddyp = bp;
        }
        else
        {
            right_buddyp = bp + csize;
            left_buddyp = bp;
        }

        // 양쪽 버디가 모두 가용 상태이고, 각 사이즈가 동일하면 (각 버디가 분할되어있지 않으면)
        if (!GET_ALLOC(HDRP(left_buddyp)) && !GET_ALLOC(HDRP(right_buddyp)) && GET_SIZE(HDRP(left_buddyp)) == GET_SIZE(HDRP(right_buddyp)))
        {
            remove_free_block(left_buddyp);
            remove_free_block(right_buddyp);        // 양쪽 버디를 모두 가용 리스트에서 제거
            csize <<= 1;                            // size를 2배로 변경
            PUT(HDRP(left_buddyp), PACK(csize, 0)); // 왼쪽 버디부터 size만큼 가용 블록으로 변경
            add_free_block(left_buddyp);            // 가용 블록으로 변경된 블록을 free list에 추가
            bp = left_buddyp;
        }
        else
            break;
    }
    return bp;
}

/*
 * First Fit: 가용 블록 리스트에서 처음으로 적합한 블록을 찾는다.
 */
static void *find_fit(size_t asize)
{
    void *bp;

    for (int class = get_class(asize); class < SEGREGATED_LIST_SIZE; class ++)
    {
        for (bp = GET_ROOT(class); bp != NULL; bp = GET_SUCC(bp))
        {
            if (asize <= GET_SIZE(HDRP(bp)))
            {
                return bp;
            }
        }
    }

    return NULL; // No fit
}

/*
 * mm_realloc - 기존 블록을 새로운 크기로 재할당한다.
 */
void *mm_realloc(void *ptr, size_t size)
{
    if (ptr == NULL) // 포인터가 NULL인 경우 할당만 수행
        return mm_malloc(size);
    if (size <= 0) // size가 0인 경우 메모리 반환만 수행
    {
        mm_free(ptr);
        return 0;
    }

    void *newptr = mm_malloc(size); // 새로 할당한 블록의 포인터
    if (newptr == NULL)
        return NULL; // 할당 실패

    size_t copySize = GET_SIZE(HDRP(ptr)) - DSIZE; // payload만큼 복사
    if (size < copySize)                           // 기존 사이즈가 새 크기보다 더 크면
        copySize = size;                           // size로 크기 변경 (기존 메모리 블록보다 작은 크기에 할당하면, 일부 데이터만 복사)

    memcpy(newptr, ptr, copySize); // 새 블록으로 데이터 복사
    mm_free(ptr);                  // 기존 블록 해제
    return newptr;
}