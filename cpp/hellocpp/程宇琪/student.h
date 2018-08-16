#ifndef STUDENT_H
#define STUDENT_H

#include <iostream>
#include <string>

namespace summercamp {

class Person
{
public:
    Person(const std::string& name="admin",int age=0)
        :_name(name),_age(age){}
    virtual ~Person(){};

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
            const std::string& school="none")
        :Person(name,age),SchoolInterface(manager),_school(school){}
    std::string& school(){return _school;}
    const std::string& school()const{return _school;}

    virtual std::string type()const{return "Student";}
    virtual std::string introduction()const{
        return type()+"[name:"+_name+", age:"
                +std::to_string(age())+", manager:"+managerName+", school:"+school()+"]";
    }
    virtual const std::string& getSchoolName()const {return _school;}
    operator std::string()const{return introduction();}
protected:
    std::string  _school;
};
inline std::ostream& operator<<(std::ostream& os,const Student& student)
{
    os<<student.introduction();
    return os;
}

}//end of namespace
#endif // STUDENT_H
