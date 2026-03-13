from pid import PID
from car import Car, Student, Animal, Cat, Dog, Mouse



car1 = Car("ferrari", 2033, "red", False)
car2 = Car("kioensegg", 2099, "green", False)


# print(car2.model)
# print(car2.color)
# print(car2.for_sale)
# print(car2.year)


car1.drive()
car2.honk()

car1.describe()





studnet1 = Student("spongebob", 112)
studnet2 = Student("patrick", 23)
studnet3 = Student("wassim", 20)


print(studnet1.name)
print(studnet2.age)
print(Student.class_year)

print(Student.num_students)


dog = Dog("scooby")
cat = Cat("garfield")
mouse = Mouse("Mickey")


print(dog.name)
print(dog.is_alive)
dog.eat()
dog.sleep()

cat.speak()
mouse.speak()