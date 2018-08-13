#ifndef _SUMMERCAMP_VECTOR_H
#define _SUMMERCAMP_VECTOR_H
#include <vector>

namespace summercamp {

    template <typename T>
    class vector
    {
    public:
        vector<T>():dataPtr_(NULL), thesize(0), thecapacity(0){};

        vector<T> (const vector<T>& init)
        {
            this->dataPtr_ = new T[init.capacity()];
            this->thesize = init.size();
            this->thecapacity = init.capacity();
            for (size_t i = 0; i < init.size(); i++)
                dataPtr_[i] = init[i];
        }

        ~vector<T>()
        {
            if (dataPtr_!=NULL)
            {
                delete[] dataPtr_;
                dataPtr_ = NULL;
            }
        }


        bool empty() const { return thesize == 0; }
        int capacity() const { return thecapacity; }
        int size() const { return thesize; }
        void clear() { thesize = 0; }

        void resize(size_t n)
        {
            if(thesize > n)
                thesize = n;
            else
            {
                thecapacity = 2*n;
                T* temPtr = new T[thecapacity];
                for (size_t i = 0; i < thesize; i++)
                    temPtr[i] = dataPtr_[i];
                delete[] dataPtr_;
                dataPtr_ = temPtr;
                thesize = n;
            }
        }
        void resize(size_t n, T init)
        {
            if(thesize > n)
                thesize = n;
            else
            {
                if (thecapacity < 2*n) {
                    thecapacity = 2 * n;
                    T *temPtr = new T[thecapacity];
                    for (size_t i = 0; i < thesize; i++)
                        temPtr[i] = dataPtr_[i];
                    for (size_t i = thesize; i < n; i++)
                        temPtr[i] = init;
                    delete[] dataPtr_;
                    dataPtr_ = temPtr;
                    thesize = n;
                }
                else
                {
                    for (size_t i = thesize; i < n; i++)
                        dataPtr_[i] = init;
                    thesize = n;
                }
            }
        }

        void reserve(size_t n)
        {
            if (n > thecapacity)
            {
                thecapacity = n;
                T *temPtr = new T[thecapacity];
                for (size_t i = 0; i < thesize; i++)
                    temPtr[i] = dataPtr_[i];
                delete[] dataPtr_;
                dataPtr_ = temPtr;
            }
        }

        void push_back(const T& item)
        {
            if (2*thesize > thecapacity)
            {
                reserve(2 * thecapacity);

            }
            dataPtr_[thesize++] = item;
        }

        T& operator[](size_t index)
        {
            return dataPtr_[index];
        }
        const T& operator[](size_t index) const
        {
            return dataPtr_[index];
        }



    private:
        T* dataPtr_;
        size_t thesize;
        size_t thecapacity;
    };


}
#endif
