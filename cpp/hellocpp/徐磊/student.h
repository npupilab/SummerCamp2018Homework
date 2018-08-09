#ifndef STUDENT_H
#define STUDENT_H

#include <iostream>
#include <string>
#include <sstream>

namespace summercamp {

class Person
{
public:
    Person(const std::string& name="admin",int age=0)
        :_name(name),_age(age){}
    virtual ~Person(){}

    virtual std::string type()const{return "Person";}
    virtual std::string introduction()const{
        return type()+"[name:"+_name+", age:"
                +std::to_string(age())+"]";
    }

    std::string& name(){return _name;}
    const std::string& name()const{return _name;}

    int& age(){return _age;}
    int  age()const{return _age;}

protected:
    std::string _name;
    int         _age;
};

class SchoolInterface
{
public:
    SchoolInterface(std::string name="manager")
        : managerName(name){
    }
    virtual const std::string& getSchoolName()const=0;

    std::string managerName;
};

class Student : public Person,public SchoolInterface
{
public:
    Student(const std::string& name="admin",int age=0,
            const std::string& manager="manager",
            const std::string& school="none"):Person(name,age),SchoolInterface(manager),_school(school){}
    virtual const std::string& getSchoolName() const
    {
        return this->_school;
    }
    virtual std::string type()const{return "Student";}
    std::string& school()
    {
        return _school;
    }
    const std::string school() const
    {
        return _school;
    }
    virtual  std::string introduction() const{
        return type()+"[name:"+_name+", age:" +std::to_string(age())+", manager:"+managerName+", school:"+_school+"]";
    }


//写法一:
//    friend std::stringstream&  operator<<(std::stringstream &is, )
//    {
//        is<<student._name<<" "<<student._age<<" "<<student.managerName<<" "<<student._school;
//        return is;
//    }
     operator std::string() const
    {
        std::stringstream ss;
        ss<<this->introduction()<<" "<<this->managerName<<" "<<this->school();
        return ss.str();
    }

protected:
    std::string  _school;
};
//写法二:
inline  std::stringstream& operator<<( std::stringstream &is, const Student& student)
 {
     is<<student.introduction()<<" "<<student.managerName<<" "<<student.school();
     return is;
 }

}//end of namespace
#endif // STUDENT_H

