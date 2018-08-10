#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <assert.h>

#include <student.h>

using namespace std;

int testStudent()
{
    using namespace summercamp;

    shared_ptr<Person> person(new Person);
    if(person->type()!="Person") return 4;
    if(person->introduction()!="Person[name:admin, age:0]")
    {
        std::cerr<<person->introduction()<<endl;
        return 4;
    }
    person=shared_ptr<Person>(new Person("zhaoyong",10));
    if(person->introduction()!="Person[name:zhaoyong, age:10]")
    {
        std::cerr<<person->introduction()<<endl;
        return 4;
    }

    person=shared_ptr<Person>(new Student("zhaoyong",10,"bushuhui","npu"));
    if(person->type()!="Student") return  3;
    if(person->name()!="zhaoyong") return 3;
    if(person->age()!=10) return 3;
    if(person->introduction()!="Student[name:zhaoyong, age:10, manager:bushuhui, school:npu]") return 3;

    shared_ptr<SchoolInterface> schoolInterface=std::dynamic_pointer_cast<SchoolInterface>(person);
    if(!schoolInterface) return 2;

    if(schoolInterface->managerName!="bushuhui") return 2;
    if(schoolInterface->getSchoolName()!="npu") return 2;

    shared_ptr<Student> student=std::dynamic_pointer_cast<Student>(person);
    if(!student) return 1;
    student->school()="nwpu";
    const Student& refStudent=*student;
    if(refStudent.school()!="nwpu") return 1;
    stringstream sst;
    sst<<refStudent;
    if(sst.str()!=string(refStudent)) return 1;


    return 0;
}

int main(int argc,char** argv){
    int ret=testStudent();
//    std::cerr<<ret;
    if(argc<4) return ret;
    std::ofstream ofs(argv[3]);
    if(!ofs.is_open()) return ret;
    string topic(argv[1]);
    string name(argv[2]);

    std::vector<std::string> results={"[S]("+topic+"/"+name+"/student.h)",
                                      "[A]("+topic+"/evaluation/funcschool.md)",
                                      "[A]("+topic+"/evaluation/school.md)",
                                      "[B]("+topic+"/evaluation/student.md)",
                                      "[B]("+topic+"/evaluation/person.md)"};
    ofs<<results[ret];
    return ret;
}

namespace demo {

/**
 * basic_ios
 * istream,ostream-> iostream
 * ifstream,ofstream  stringstream
 */
class Point{
public:
    double x,y,z;
};

std::ostream& operator<<(std::ostream& os,const Point& p)
{
    os<<p.x<<" "<<p.y<<" "<<p.z;
    return os;
}

std::istream& operator>>(std::istream& is,Point& p)
{
    is>>p.x>>p.y>>p.z;
    return is;
}

int testStream()
{
    std::cout<<"cout"<<std::endl;
    std::cerr<<"cerr"<<std::endl;
    int num;
    std::cin>>num;//istream
    std::cout<<"num:"<<num;

    std::ifstream ifs("in.txt");
    std::ofstream ofs("out.txt");
    if(!ifs.is_open())
        return -1;

    ifs>>num;
    ofs<<num;

    std::stringstream sst("");
    sst<<num;
    sst>>num;
    return 0;
}

class Person
{
public:
    Person(std::string name="admin",int age=0)
        : _name(name),_age(age){
        std::cerr<<"create Person"<<std::endl;
    }
    ~Person(){
      std::cerr<<"Object "<<_name<<" are leaving.";
    }

    virtual std::string introduction()const{
        std::stringstream sst;
        sst<<"name:"<<_name<<", age:"<<_age;
        return sst.str();
    }

    const std::string& name()const{return _name;}
    std::string& name(){return _name;}

protected:
    std::string _name;
public:
    int         _age;
};

class SchoolInterface
{
public:
    SchoolInterface(std::string name="manager")
        :managerName(name){}
    virtual std::string getSchoolName()const{return "none";}

    std::string managerName;
    int         fdalfda;
};

class Student : public Person,public SchoolInterface{
public:
    Student(std::string name="admin",
            int age=0,std::string school="world");

    Student(Person person,std::string school="world");

    std::string school(){return _school;}

    virtual std::string introduction()const{
        std::stringstream sst;
        sst<<Person::introduction()
          <<", school:"<<_school;
        return sst.str();
    }

    virtual std::string getSchoolName()const{return _school;}

    std::string _school;
};

Student::Student(std::string name, int age, std::string school)
    : Person(name,age),_school(school){

}

Student::Student(Person person,std::string school)
    : Person(person),_school(school)
{

}

int demoMain(int,char**){
    Person  person("zhaoyong",12);
    Student student("zhaoyong",12,"npu");

    Person* p1=&person;
    Person* p2=&student;

    SchoolInterface* interface=dynamic_cast<SchoolInterface*>(&student);
    Student* stu=dynamic_cast<Student*>(p1);

    auto school=stu?stu->school():std::string("NoneSchool");
    std::cerr<<p1->introduction()<<std::endl;
    std::cerr<<p2->introduction()<<std::endl;
    std::cerr<<school<<std::endl;
    std::cerr<<interface->managerName<<std::endl;

    Person* p3(new Person());
    Person* p4(new Student());
    std::cerr<<p3->introduction()<<std::endl;
    std::cerr<<p4->introduction()<<std::endl;

    double* p=new double(1);
    double* ptr=(double*)malloc(sizeof(double));
    delete  p;
    delete  ptr;

    int n;
    std::cin>>n;
    double* pmulti=(double*)malloc(n*sizeof(double));
    double* ptrmulti=new double[n];
    free(pmulti);
    delete[] ptrmulti;

    Person* persons=new Person[2];
    delete[] persons;
    std::cerr<<"released";

    std::shared_ptr<Person> personPtr(new Student("newstudent"));


    Person &ref=person,&ref1=student;
    ref._age=100;
    ref1._age=120;
    std::cerr<<person._age<<","<<student._age;

    std::cerr<<ref.introduction();
    std::cerr<<ref1.introduction();

    std::cerr<<Point();

    {
        const Person person;
        std::string name=person.name();
    }

    return 0;
}


}
