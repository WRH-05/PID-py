
class Car:
    def __init__(self, model, year, color, for_sale):
        self.model = model
        self.color = color
        self.year = year
        self.for_sale = for_sale

    def drive(self):
        print(f"you're driving the {self.model}")

    def stop(self):
        print(f"you've stopped the {self.color} {self.model}")

    def honk(self):
        print("honk honk!")

    def describe(self):
        print(f"This car is a {self.model} it's the {self.year} model, it's {self.color} color shines brighter then ever")




class Student:

    class_year = 2024
    num_students = 0

    def __init__(self, name, age):
        self.name = name
        self.age = age
        Student.num_students += 1



class Animal:

    def __init__(self, name):
        self.name = name
        self.is_alive = True
    
    def eat(self):
        print(f"{self.name} is eating")

    def sleep(self):
        print(f"{self.name} is sleeping")    


class Dog(Animal):
    
    def speak(self):
        print("woof!")

class Cat (Animal):
        
    def speak(self):
        print("meow!")

class Mouse(Animal):
    
    def speak(self):
        print("squeak!")


from abc import ABC, abstractmethod


class Body(ABC):

    

    @abstractmethod
    def think(self):
        pass

    @abstractmethod
    def sleep(self):
        pass


