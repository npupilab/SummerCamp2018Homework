#!/usr/bin/python27
#coding:utf-8

print "hello "
var1='hello python'
var2='python runoob'

print "var1[0]: ",var1[0]
print "var2[1:5]:",var2[1:5]

print "my name is %s and weight is %d kg!"%('zara',21)

list1=['huboni','hello']
list2=[1,2,3,4,5,6,7]
list3=['a','b','c','d']

print 'list1[0]:',list1[0]
print 'list2[1:5]:',list2[1:5]

#sum
s=0
for x in list2:
    s=s+x;
print s


list5=[1,2,3,7]
list6=[2,5,67,8]
print (cmp(list5,list6))

tuple=(['physics','chemicla'],'math','chinese')
tuple[0].append('afas')
print tuple

print list(tuple)

dict={'name':'zefra','age':7,'tongpoinm':100}
print dict
print dict.items()

#连接tuple
tup1=(12,34.56)
tup2=('abd','aff')
tup3=tup1+tup2
print tup3

dict={'name':'zara','age':7,'class':'firday'}
print dict['name']
print dict['age']

dict['age']=8
dict['school']='dps sdhaog'
print dict


a=60
b=13
c=0

c=a&b
print c

a=21
b=10
c=0

print

