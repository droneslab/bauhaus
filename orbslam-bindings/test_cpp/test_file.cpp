#include <iconv.h>
#include <iostream>
#include <stdio.h>
using namespace std;

#define BUF_SIZE 1024
size_t z = (size_t) BUF_SIZE-1;

bool sjis2utf8( char* text_sjis, char* text_utf8 )
{
  iconv_t ic;
  ic = iconv_open("UTF8", "SJIS"); // sjis->utf8
  iconv(ic , &text_sjis, &z, &text_utf8, &z);
  iconv_close(ic);
  return true;
}
int main(void)
{
  char hello[BUF_SIZE] = "hello";
  char bye[BUF_SIZE] = "bye";
  char tmp[BUF_SIZE] = "something else";

  sjis2utf8(hello, tmp);
  cout << tmp << endl;

  sjis2utf8(bye, tmp);
  cout << tmp << endl;
}
