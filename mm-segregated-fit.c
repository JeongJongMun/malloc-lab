/*
 * < mm-segregated-fit.c >
 * 이 방법은 분리 가용 리스트(Segregated Free List)를 사용하여 메모리 할당을 관리하는 malloc 패키지를 구현하였다.
 * 다수의 명시적 가용 리스트를 유지하며, 각 리스트는 2^k의 크기 클래스로 구분된다.
 * 각 클래스는 2^k 이상 2^(k+1) 미만의 크기를 가진 블록을 저장하고, 분리 맞춤(Segregated Fit)방식을 사용한다.
 * 이 방법에서는 더블 워드 정렬을 사용하며, 모든 블록의 헤더와 풋터에 크기와 할당 비트를 저장한다. (1 Word = 4 Byte)
 * 최소 블록 크기는 4 Word (16 Byte)이며, 헤더와 풋터는 각각 1 Word이다.
 *
 * 사용 가능한 메모리 할당 정책은 다음과 같다.
 * 1. First Fit: 가용 블록 리스트에서 처음으로 적합한 블록을 찾는다.
 * 2. Best Fit: 가용 블록 리스트에서 가장 작은 적합한 블록을 찾는다.
 *
 * 명시적 가용 리스트와 다른 부분은 다음과 같다.
 * 1. 매크로 추가 : SEGREGATED_LIST_SIZE, GET_ROOT
 * 2. mm_init()
 * 3. remove_free_block()
 * 4. add_free_block()
 * 5. get_class() 추가
 * 6. find_fit()
 * 7. free_listp -> heap_listp로 변경
 *
 */

/*
 * < 분리 가용 리스트(Segregated Free List) >
 *
*                                    heap_listp
*                                        |
*                                        |
*                                        |
*                                        V
 * ---------------------------------------
 * | Alignment Padding | Prologue Header | . . . . 아래에 이어서 그리겠음
 * ---------------------------------------
*                                                                                                                                               mem_brk
*                                                                                                                                                  |
*                                                                                                                                                  |
*                                                                                                                                                  |
*                                                                                                                                                  V
 * -------------------------------------------------------------------------------------------------------------------------------------------------
 * Segregate Free List Root | ... | Segregate Free List Root | Prologue Footer | Free Block | ... | Free Block | ...| Free Block | Epilogue Header |
 * -------------------------------------------------------------------------------------------------------------------------------------------------
 *
 * Alignment Padding: 8의 배수로 맞추기 위한 패딩 (값 = 0)
 * Prologue Header: 가용 블록의 시작을 나타내는 헤더 (값 = 8, 할당 비트 = 1)
 * Segregate Free List Root: 각각의 크기 클래스에 대한 가용 리스트의 루트 (2^k)
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
 * |                                       |
 * -----------------------------------------
 * |           Padding(Optional)           |
 * -----------------------------------------
 * | Block size                | alloc bit | Footer
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
 * | Block size                | alloc bit | Footer
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

// #define FIRST_FIT // 48 + 40 = 88
#define BEST_FIT // 52 + 40 = 92
// #define WORST_FIT // 45 + 40 = 85

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
// 블록 포인터의 풋터 주소를 반환
#define FTRP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

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
// 가용 리스트의 개수 2^20 = 1,048,576까지
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
    PUT(heap_listp, 0);                                                                                  // Alignment Padding
    PUT(heap_listp + (1 * WSIZE), PACK((SEGREGATED_LIST_SIZE + 2) * WSIZE, 1));                          // Prologue Header
    for (int i = 0; i < SEGREGATED_LIST_SIZE; i++)                                                       // Segragated Free List 초기화
        PUT(heap_listp + ((2 + i) * WSIZE), NULL);                                                       // 각각의 Segregated Free List Root
    PUT(heap_listp + ((SEGREGATED_LIST_SIZE + 2) * WSIZE), PACK((SEGREGATED_LIST_SIZE + 2) * WSIZE, 1)); // Prologue Footer
    PUT(heap_listp + ((SEGREGATED_LIST_SIZE + 3) * WSIZE), PACK(0, 1));                                  // Epilogue Header

    heap_listp += (2 * WSIZE); // 첫 번째 분리 가용 블록의 시작 주소

    /* CHUNKSIZE만큼의 가용 블록으로 초기 힙을 확장한다.
     * 16을 더해주는 이유는 4번 테스트 케이스로 처음에 4,095와 같은 값이 들어왔을 때,
     * 추가적인 힙 확장 없이 할당을 하여 공간을 더 효율적으로 사용할 수 있다.
     */
    if (extend_heap((CHUNKSIZE + 2 * DSIZE) / WSIZE) == NULL)
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

    // 새로운 가용 블록의 헤더와 풋터를 초기화하고 새로운 에필로그 헤더를 설정
    PUT(HDRP(bp), PACK(size, 0));         // 가용 블록 헤더
    PUT(FTRP(bp), PACK(size, 0));         // 가용 블록 풋터
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1)); // 새로운 에필로그 헤더

    // 이전 블록이 가용 상태이면 통합
    return coalesce(bp);
}

/*
 * mm_malloc - 블럭을 할당하고 적합한 블록을 찾지 못했을 때 힙을 확장한다.
 * 블록의 크기는 최소 16바이트 크기의 블록으로 구성
 * 8바이트는 정렬 조건을 만족하기 위해, 8바이트는 헤더와 풋터를 위해 사용
 */
void *mm_malloc(size_t size) // size는 헤더와 풋터를 제외한 블록의 크기
{
    size_t asize;      // 헤더와 풋터를 포함한 조정된 블록의 크기
    size_t extendsize; // 적합한 블록을 찾지 못했을 때 힙을 확장하는 양
    char *bp;

    // 가짜 블록을 할당하지 않음
    if (size == 0)
        return NULL;

    if (size <= DSIZE)
        asize = 2 * DSIZE;
    else
        asize = ALIGN(size + DSIZE); // 헤더와 풋터를 포함하여 8의 배수로 올림

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
    PUT(FTRP(bp), PACK(size, 0));
    coalesce(bp);
}

/*
 * place - 가용 블록의 시작 부분에 asize 바이트의 블록을 배치하고 나머지가 최소 블록 크기 이상이면 분할한다.
 */
static void place(void *bp, size_t allocate_size) // allocate_size는 헤더와 풋터를 포함한 블록의 크기
{
    remove_free_block(bp); // 명시적 가용 리스트에서 가용 블록을 제거
    size_t chunk_size = GET_SIZE(HDRP(bp));

    if ((chunk_size - allocate_size) >= (2 * DSIZE))
    { // 가용 블록에 할당을 하고 남은 공간이 최소 블록 크기 이상이면 분할
        PUT(HDRP(bp), PACK(allocate_size, 1));
        PUT(FTRP(bp), PACK(allocate_size, 1));
        bp = NEXT_BLKP(bp);
        PUT(HDRP(bp), PACK(chunk_size - allocate_size, 0));
        PUT(FTRP(bp), PACK(chunk_size - allocate_size, 0));
        add_free_block(bp); // 명시적 가용 리스트에 가용 블록을 추가
    }
    else
    { // 가용 블록에 할당을 하고 남은 공간이 최소 블록 크기보다 작으면 분할하지 않음
        PUT(HDRP(bp), PACK(chunk_size, 1));
        PUT(FTRP(bp), PACK(chunk_size, 1));
    }
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
    if (size < 16) // 최소 블록 크기는 16바이트
        return -1;

    // 클래스별 최소 크기
    size_t class_sizes[SEGREGATED_LIST_SIZE] = {
        16,
    };

    // 주어진 크기에 적합한 클래스 검색
    for (int i = 1; i < SEGREGATED_LIST_SIZE; i++)
    {
        class_sizes[i] = class_sizes[i - 1] << 1;
        if (size <= class_sizes[i])
            return i;
    }

    // 주어진 크기가 마지막 클래스의 범위를 넘어갈 경우, 마지막 클래스로 처리
    return SEGREGATED_LIST_SIZE - 1;
}

/*
 * coalesce - 경계 태그 합치기. 합쳐진 블록의 포인터를 반환한다.
 */
static void *coalesce(void *bp)
{
    size_t prev_alloc = GET_ALLOC(FTRP(PREV_BLKP(bp))); // 이전 블록의 할당 상태 (0 or 1)
    size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp))); // 다음 블록의 할당 상태 (0 or 1)
    size_t size = GET_SIZE(HDRP(bp));                   // 현재 블록의 크기

    if (prev_alloc && next_alloc)
    { // Case 1: 이전 블록과 다음 블록이 모두 할당 상태
        add_free_block(bp);
        return bp;
    }
    else if (prev_alloc && !next_alloc)
    { // Case 2: 이전 블록은 할당 상태이고 다음 블록은 가용 상태
        remove_free_block(NEXT_BLKP(bp));
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(bp), PACK(size, 0));
        PUT(FTRP(bp), PACK(size, 0));
    }
    else if (!prev_alloc && next_alloc)
    { // Case 3: 이전 블록은 가용 상태이고 다음 블록은 할당 상태
        remove_free_block(PREV_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp)));
        PUT(FTRP(bp), PACK(size, 0));
        PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));
        bp = PREV_BLKP(bp);
    }
    else
    { // Case 4: 이전 블록과 다음 블록이 모두 가용 상태
        remove_free_block(PREV_BLKP(bp));
        remove_free_block(NEXT_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp))) + GET_SIZE(FTRP(NEXT_BLKP(bp)));
        PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(size, 0));
        bp = PREV_BLKP(bp);
    }
    add_free_block(bp);

    return bp;
}

#if defined(FIRST_FIT)
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

#elif defined(BEST_FIT)
/*
 * Best Fit: 가용 블록 리스트에서 가장 작은 적합한 블록을 찾는다.
 */
static void *find_fit(size_t asize)
{
    void *bp;
    void *best_bp = NULL;
    size_t min_size = 0;

    for (int class = get_class(asize); class < SEGREGATED_LIST_SIZE; class ++)
    {
        for (bp = GET_ROOT(class); bp != NULL; bp = GET_SUCC(bp))
        {
            if (asize <= GET_SIZE(HDRP(bp)))
            {
                if (min_size == 0 || GET_SIZE(HDRP(bp)) < min_size)
                {
                    min_size = GET_SIZE(HDRP(bp));
                    best_bp = bp;
                }
            }
        }
    }
    return best_bp;
}

#elif defined(WORST_FIT)
/*
 * Worst Fit: 가용 블록 리스트에서 가장 큰 적합한 블록을 찾는다.
 */
static void *find_fit(size_t asize)
{
    void *bp;
    void *worst_bp = NULL;
    size_t max_size = 0;

    for (int class = get_class(asize); class < SEGREGATED_LIST_SIZE; class ++)
    {
        for (bp = GET_ROOT(class); bp != NULL; bp = GET_SUCC(bp))
        {
            if (asize <= GET_SIZE(HDRP(bp)))
            {
                if (max_size == 0 || GET_SIZE(HDRP(bp)) > max_size)
                {
                    max_size = GET_SIZE(HDRP(bp));
                    worst_bp = bp;
                }
            }
        }
    }
    return worst_bp;
}
#endif

/*
 * mm_realloc - 기존 블록을 새로운 크기로 재할당한다.
 */
void *mm_realloc(void *ptr, size_t size)
{
    void *oldptr = ptr; // 이전 포인터
    void *newptr;       // 새로 메모리 할당할포인터

    size_t originsize = GET_SIZE(HDRP(oldptr)); // 원본 사이즈
    size_t newsize = size + DSIZE;              // 새 사이즈

    if (size == 0)
        return NULL;

    // 새로운 사이즈가 원본 사이즈보다 작거나 같으면
    if (newsize <= originsize)
        return oldptr;
    else
    {                                                                    // 기존 블록의 다음 블록이 가용 블록이고, 추가 사이즈가 충분하면
        size_t addSize = originsize + GET_SIZE(HDRP(NEXT_BLKP(oldptr))); // 추가 사이즈 -> 헤더 포함 사이즈
        if (!GET_ALLOC(HDRP(NEXT_BLKP(oldptr))) && (newsize <= addSize))
        {                                         // 가용 블록이고 사이즈 충분
            remove_free_block(NEXT_BLKP(oldptr)); // 다음 블록 제거
            PUT(HDRP(oldptr), PACK(addSize, 1));  // 새로운 헤더
            PUT(FTRP(oldptr), PACK(addSize, 1));  // 새로운 푸터
            return oldptr;
        }
        else // 새로운 블록 할당
        {
            newptr = mm_malloc(newsize);
            if (newptr == NULL)
                return NULL;
            memcpy(newptr, oldptr, newsize);
            mm_free(oldptr);
            return newptr;
        }
    }
}