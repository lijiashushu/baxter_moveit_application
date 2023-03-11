//
// Created by lijiashushu on 20-10-7.
//

#include <stdio.h>
#include <string.h>


#define MAX_BUF  256

unsigned char getBit(unsigned char, int);
unsigned char setBit(unsigned char, int);
unsigned char clearBit(unsigned char, int);
void encrypt(unsigned char* , unsigned char, unsigned char, int);
void translate(char* , unsigned char, unsigned char, int);

int main()
{
  char str[8];
  int  choice;

  printf("\nYou may:\n");
  printf("  (1) Encrypt a message \n");
  printf("  (2) Decrypt a message \n");
  printf("\n  what is your selection: ");
  fgets(str, sizeof(str), stdin);
  sscanf(str, "%d", &choice);

  unsigned char key = 0b10110010;
  unsigned char counter = 0b00101000;



    switch (choice) {
    case 1:
    {
      char plaintext[MAX_BUF];
      fgets(plaintext, sizeof(plaintext), stdin);
      int len = strlen(plaintext);
      translate(plaintext, key, counter, len);
      for (int i = 0; i < len; i++)
      {
        printf("%d ", plaintext[i]);
      }
      printf("%d\n", -1);
      break;
    }


    case 2:
    {
      char ciphertext[MAX_BUF];
      int len = 0;
      int input;
      while (1)
      {
        scanf("%d", &input);
        if (input != -1)
        {
          ciphertext[len++] = input;
        }
        else
        {
          break;
        }
      }
      translate(ciphertext, key, counter, len);
      ciphertext[len] = '\0';
      printf("%s", ciphertext);
      break;
    }
    default:

      break;
  }

  return 0;
}



/*
  Function:  getBit
  Purpose:   retrieve value of bit at specified position
       in:   character from which a bit will be returned
       in:   position of bit to be returned
   return:   value of bit n in character c (0 or 1)
*/
unsigned char getBit(unsigned char c, int n)
{
  unsigned char res = c & (1 << n) ? 1 : 0;
  return res;
}

/*
  Function:  setBit
  Purpose:   set specified bit to 1
       in:   character in which a bit will be set to 1
       in:   position of bit to be set to 1
   return:   new value of character c with bit n set to 1
*/
unsigned char setBit(unsigned char c, int n)
{
  c |= 1 << n;
  return c;
}

/*
  Function:  clearBit
  Purpose:   set specified bit to 0
       in:   character in which a bit will be set to 0
       in:   position of bit to be set to 0
   return:   new value of character c with bit n set to 0
*/
unsigned char clearBit(unsigned char c, int n)
{
  c &= ~(1 << n);
  return c;
}

unsigned char update_counter(unsigned char key, unsigned char counter)
{
  unsigned char temp = counter;
  for (int i = 7; i >=0; i--)
  {
     int j;
     if (getBit(key, i) == 1)
     {
       j = (i + 1) % 8;
     }
     else
     {
       j = ((i - 1) + 8) % 8;
     }
     unsigned char res = getBit(temp, i) ^ getBit(temp, j);
     if (res == 0)
     {
       temp = clearBit(temp, i);
     }
     else
     {
       temp = setBit(temp, i);
     }
  }
  return temp;
}

char trans_character(char c, unsigned char counter)
{
  char temp = c;
  unsigned char mod = counter % 9;
  if (mod < 3)
  {
    for (int i = 0; i < 8; i+=2)
    {
      int xor_res = getBit(counter, i) ^ getBit(c, i);
      if (xor_res == 0)
      {
        temp = clearBit(temp, i);
      }
      else
      {
        temp = setBit(temp, i);
      }
    }

  }
  else if (mod >= 3 && mod <= 5)
  {
    for (int i = 1; i < 8; i+=2)
    {
      int xor_res = getBit(counter, i) ^ getBit(c, i);
      if (xor_res == 0)
      {
        temp = clearBit(temp, i);
      }
      else
      {
        temp = setBit(temp, i);
      }
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      int xor_res = getBit(counter, i) ^ getBit(c, i);
      if (xor_res == 0)
      {
        temp = clearBit(temp, i);
      }
      else
      {
        temp = setBit(temp, i);
      }
    }
  }
  return temp;
}

void translate(char* str, unsigned char key, unsigned char counter, int len)
{
  for (int i = 0; i < len; i++){
    counter = update_counter(key, counter);
    ++counter;
    ++key;
    str[i] = trans_character(str[i], counter);
  }
}




