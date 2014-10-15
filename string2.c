#include "string2.h"

char* sp = NULL; /* the start position of the string */

char* strtok(char* str, const char* delimiters) {

    int i = 0;
    int len = strlen(delimiters);

    /* check in the delimiters */
//    if(len == 0)
//        printf("delimiters are empty\n");

    /* if the original string has nothing left */
    if(!str && !sp)
        return NULL;

    /* initialize the sp during the first call */
    if(str && !sp)
        sp = str;

    /* find the start of the substring, skip delimiters */
    char* p_start = sp;
    while(1) {
        for(i = 0; i < len; i ++) {
            if(*p_start == delimiters[i]) {
                p_start ++;
                break;
            }
        }

        if(i == len) {
               sp = p_start;
               break;
        }
    }

    /* return NULL if nothing left */
    if(*sp == '\0') {
        sp = NULL;
        return sp;
    }

    /* find the end of the substring, and
        replace the delimiter with null */
    while(*sp != '\0') {
        for(i = 0; i < len; i ++) {
            if(*sp == delimiters[i]) {
                *sp = '\0';
                break;
            }
        }

        sp ++;
        if (i < len)
            break;
    }

    return p_start;
}

size_t strlen(const char *s) {
    const char *start = s;
    while(*s)
        s++;
    return s - start;
}
