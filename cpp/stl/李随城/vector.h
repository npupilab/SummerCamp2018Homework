#ifndef _SUMMERCAMP_VECTOR_H
#define _SUMMERCAMP_VECTOR_H
#include <vector>

namespace summercamp {

template <class T>
class vector
{
private:
    size_t my_size, my_capacity;
    T* my_vector = NULL;

public:
    vector()
    {
        my_size = 0;
        my_capacity = 0;
    }
    vector(size_t n)
    {
        my_vector = new T[n];
        my_size = n;
        my_capacity = n;
    }
    vector(size_t n, T data)
    {
        my_vector = new T[n];
        for(size_t i = 0; i < n; i++)
        {
            my_vector[i] = data;
        }

        my_size = n;
        my_capacity = n;

    }
    ~vector()
    {
        if(my_vector != NULL)
            delete[] my_vector;
    }

    size_t capacity(){return my_capacity;}
    size_t size(){return my_size;}
    bool empty(){return my_size==0;}
    void push_back(T newdata);
    void resize(size_t n, T newdata);
    void clear();
    T& operator[](const size_t index){return my_vector[index];}
    const T& operator[](const size_t index) const{return my_vector[index];}
    void reserve(size_t n);
};

template <class T>
void vector<T>::reserve(size_t n)
{
    if(n > my_capacity)
    {
        T* new_capacity = new T[n]();
        for(size_t i = 0; i < my_size; i ++)
        {
            new_capacity[i] = my_vector[i];
        }

        delete[] my_vector;
        my_vector = new_capacity;
        my_capacity = n;
    }
    else
    {
        return;
    }

}

template <class T>
void vector<T>::push_back(T newdata)
{
    if(my_capacity > my_size)
    {
        my_vector[my_size] = newdata;
        my_size += 1;
    }
    else
    {
        my_capacity = 2 * my_size;
        T* new_capacity = new T[my_capacity]();
        for(size_t i = 0; i < my_size; i ++)
        {
            new_capacity[i] = my_vector[i];
        }

        delete[] my_vector;
        my_vector = new_capacity;
        my_vector[my_size] = newdata;
        my_size += 1;
    }
}

template <class T>
void vector<T>::resize(size_t n, T newdata)
{
    T* new_capacity = new T[n]();
    for(size_t i = 0; i < my_size; i ++)
    {
        new_capacity[i] = my_vector[i];
    }

    for(size_t i = my_size; i < n; i++)
    {
        new_capacity[i] = newdata;
    }

    delete[] my_vector;
    my_vector = new_capacity;
    my_size = n;
}

template <class T>
void vector<T>::clear()
{
    T* new_capacity = new T[my_capacity]();
    delete[] my_vector;
    my_vector = new_capacity;

    my_size = 0;
}


};

#endif
