#ifndef _SUMMERCAMP_VECTOR_H
#define _SUMMERCAMP_VECTOR_H
#include <vector>
namespace summercamp {

template <typename T>
class vector
{
public:
    vector<T>():dataPtr_(NULL),size_(0),capacity_(0){}
    vector<T>(size_t n)
    {
        dataPtr_ = new T[n];
        size_ = n;
    }
    vector<T>(size_t n, T init)
    {
        dataPtr_ = new T[n](T(init));
        size_ = size_;
        capacity_ = n;
    }
    vector<T>(const vector<T>& init)
    {
        this->dataPtr_ = new T[init.capacity()];
        this->size_ = init.size();
        this->capacity_ = init.capacity();
        for(size_t i = 0; i< size_;i++) dataPtr_[i] = init[i];
    }

   ~vector<T>()
    {
        if(dataPtr_!=NULL)
        {
            delete [] dataPtr_;
            dataPtr_ =NULL;
        }
    }



    vector<T> operator =(const vector<T>& other)
    {
        if(dataPtr_ != NULL)
        {
            delete [] dataPtr_;
            dataPtr_ =NULL;
        }
        if(other.size() == 0)
        {
            size_ = 0;
            capacity_ = 0;
            dataPtr_ = NULL;
        }
        else
        {
            size_ = other.size();
            capacity_ = other.capacity();
            dataPtr_ = new T[capacity_];
            for(size_t i =0 ; i< size_; i++) dataPtr_[i] = other[i];
        }
        return *this;
    }



    T& operator[](size_t index)
    {
        return dataPtr_[index];
    }
    const T& operator[](size_t index) const
    {
        return dataPtr_[index];
    }

    void resize(size_t n)
    {
        if(size_ > n)
        {
            size_ = n;
        }
        else
        {
            capacity_  = 2*n;
            T *temPtr =  new T[capacity_];
            for(size_t i=0; i< size_;i++) temPtr[i] = dataPtr_[i];
            delete[] dataPtr_;
            dataPtr_ = temPtr;
            size_ = n;
        }
    }
    void resize(size_t n, T init)
    {
        if( n <= size_ )
        {
            size_ = n;
            return ;
        }
        else
        {
            if(capacity_ < 2*n)
            {
                capacity_  = 2*n;
                T *temPtr =  new T[capacity_];
                for(size_t i=0; i< size_;i++) temPtr[i] = dataPtr_[i];
                for(size_t i=size_; i < n ;i++) temPtr[i] = init;
                size_ = n;
                delete[] dataPtr_;
                dataPtr_ = temPtr;
            }
            else
            {
              for(size_t i=size_; i < n ;i++) dataPtr_[i] = init;
              size_ = n;
            }

        }
    }

    void reserve(size_t n)
    {
        if(n > capacity_)
        {
            capacity_ = n;
            T *tempPtr = new T[capacity_];
            for(size_t i=0; i< size_;i++) tempPtr[i] = dataPtr_[i];
            delete[] dataPtr_;
            dataPtr_ = tempPtr;
        }
    }

    void push_back(const T& item)
    {
        if(2*size_ > capacity_)
        {
            capacity_ = 2*size_;
            size_+=1;
            T *tempPtr = new T[capacity_];
            size_t i;
            for(i=0;i<size_;i++) tempPtr[i] = dataPtr_[i];
            dataPtr_[i] = item;
        }
        else
        {
            dataPtr_[size_] = item;
            size_+=1;
        }
    }
    size_t size() const
    {
        return size_;
    }

    size_t capacity() const
    {
        return capacity_;
    }

    bool empty() const
    {
        return size_==0;
    }

    void clear()
    {
        size_ = 0;
    }

private:
    T * dataPtr_;
    size_t size_;
    size_t capacity_;


};

}
#endif

