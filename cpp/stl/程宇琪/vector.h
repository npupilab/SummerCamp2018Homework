#ifndef _H_VECTOR
#define _H_VECTOR

#include <iostream>
#include <vector>

namespace summercamp {

template <typename T>
class vector {
 public:
  vector() : m_size(0), m_Max(0), m_data(NULL){};
  vector(int size,T data):m_size(size),m_data(new T[size]), m_Max(size){
      m_data[0] = data;
  }

  vector(const vector& arr)
  {
      m_Max = arr.m_size;
      m_size = arr.m_size;
      m_data = new T[m_size];
      for(int i = 0; i < m_size; i++)
      {
          m_data[i] = arr.m_data[i];
      }
  }
  ~vector(){
      Free();
  }

  int size(){
      return m_size;
  }
  int capacity(){
      return m_Max;
  }
  void reserve(int max){
      if(max>m_Max)
      {
          T *newdata = new T[max];
          for(int i = 0 ; i < m_size ; i++)
          {
              newdata[i] = m_data[i];
          }
         /* for(int i = m_size ; i < max ; i++)
          {
              newdata[i] = (T) 0;
          }*/

          delete []m_data;
          m_data = newdata;
          m_Max = max;

      }

  }
  bool empty(){
      if(m_size)
          return 0;
      else
          return 1;
  }
  void push_back(T data)
  {
      if(m_Max == 0)
      {
          reserve(2);
      }
      if(m_size == m_Max)
      {
          reserve(2 * m_Max);
      }
      *(m_data + m_size) = data;
      m_size++;
  }
  void resize(int size, T data = (T)0)
  {
          push_back(data);
          if(size > m_Max)
          {
              reserve(size);
          }
          m_size = size;
  }
  T& operator [] (int index) const
  {
      if(judgeindex(index))
      {
          return *(m_data + index);
      }
  }
  void clear(){
      m_size = 0;
  }



 private:
  T *m_data;
  int m_size;
  int m_Max;

 private:
  void Init();
  void Free() {
    delete[] m_data;
    m_data = NULL;
  }
  inline int judgeindex(int nindex)const{
      if(nindex > m_size)
          return 0;
      else return 1;
  }

};
}

#endif
