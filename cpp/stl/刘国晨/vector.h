#ifndef _SUMMERCAMP_VECTOR_H
#define _SUMMERCAMP_VECTOR_H
#include <vector>

namespace summercamp {

template <typename T>
class vector
{
public:
    vector():m_nSize(0), m_pData(NULL), m_nMax(0){}// 缺省构造函数
    vector(int nSize, T dValue):m_nSize(nSize), m_pData(new T[nSize]), m_nMax(nSize){}
    ~vector(){Free();}

    int capacity(){return m_nMax;}
    int size(){return m_nSize;}
    bool empty(){return m_nSize?0:1;}

    bool reserve(int capacity)
    {
        if(capacity <= m_nMax)
           return 1;
        else if(capacity > m_nMax)
        {
            T* newdata = new T[capacity];

            for(int i = 0; i < m_nSize; ++i)
            {
                *(newdata+i) = *(m_pData+i);
            }

            delete []m_pData;
            m_pData = newdata;
            newdata = NULL;
            m_nMax = capacity;

            return 1;
        }
    }

    bool push_back(T data)
    {
        if(m_nMax == 0)
        {
            resize(2);
        }

        else if(m_nSize == m_nMax)
        {
            resize(2 * m_nMax);
        }

        *(m_pData + m_nSize) = data;
        m_nSize++;
        return 1;
    }

    bool resize(int nSize)
    {
        T *newdata = new T[nSize];

        if(nSize > m_nSize)
        {
            for(int i = 0; i < m_nSize; i++)
            {
                *(newdata+i) = *(m_pData+i);
            }

            for(int i = m_nSize; i < nSize; i++)
            {
                *(newdata+i) = (T)0;
            }
        }

        else
        {
            for(int i = 0; i < nSize; i++)
            {
                *(newdata+i) = *(m_pData+i);
            }

            m_nSize = nSize;
        }

        delete []m_pData;
        m_pData = newdata;
        newdata = NULL;
        m_nMax = nSize;
        return 1;
    }

    bool resize(int nSize, T data)
    {
        if(nSize > m_nMax)
        {
            T *newdata = new T[nSize];

            for(int i = 0; i < m_nSize; i++)
            {
                *(newdata+i) = *(m_pData+i);
            }

            for(int i = m_nSize; i < nSize; i++)
            {
                *(newdata+i) = data;
            }

            delete []m_pData;
            m_pData = newdata;
            newdata = NULL;
            m_nMax = nSize;
        }
        else if(nSize >= m_nSize)
        {
            for(int i = m_nSize; i < nSize; i++)
            {
                *(m_pData + i) = data;
            }
        }
        else if(nSize < m_nSize)
        {
            for(int i = nSize; i > m_nSize; --i)
            {
                delete (m_pData + i);
            }
        }

        m_nSize = nSize;
        return 1;
    }

    T& operator[] (int nIndex) const // 重载[]操作符，以便像传统数组那样通过a[k]来获取元素值
    {
        if(InvalidateIndex(nIndex))
        {
            return *(m_pData + nIndex);
        }
    }

    bool clear()
    {
        for(int i = 0; i < m_nSize; ++i)
            delete (m_pData + i);
    }


private:
    T *m_pData;// 存放数组的动态内存指针
    int m_nSize;// 数组的元素个数
    int m_nMax;//预留给动态数组的内存大小

private:
    void Init();	// 初始化
    void Free()// 释放动态内存
    {
        delete []m_pData;
        m_pData = NULL;
    }

    inline int InvalidateIndex(const int nIndex) const	// 判断下标的合法性
    {
        if(nIndex >= m_nSize)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
};

}
#endif
