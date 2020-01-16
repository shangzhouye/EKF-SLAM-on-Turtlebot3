# Answers to C.002

1. There are a few major differences.

- A struct is a bundle to store data, while a class has a higher level of abstraction. A class can both store data, and be an interface to perform actions.
- A class is able to combine data and methods (functions) into an abstract object.
- A class has the capability to be inherited.
- Members of a class are private by default and members of a struct are public by default.

2. Why is Vector2D a struct and Transform2D Class? 

- "C.8: Use class rather than struct if any member is non-public"
  - Transform2D has non-public members (the translation and rotation component).
- "Use class if the class has an invariant; use struct if the data members can vary independently"
  - components of Vector2D can vary independently, thus using struct is reasonable considering its convenience.

3. Why are some of the constructors in Transform2D explicit?

- "By default, declare single-argument constructors explicit"
  - Single-argument constructors can perform implicit type conversion. To avoid that unintended conversion, `explicit` is prefered.

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D).

- It be either passed by copy, by reference or by pointer.
- Pass by copy would ensure the original vector would not be modified. Pass by reference or pointer can save the memory but can possibly modified the original vector.
- Pass by copy as the vector does not occupy much memory. (There is also the possibility to pass by constant reference.)

5. Implement the normalize functionality using the method you chose.

6. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?

- An object declared as const cannot be modified so that accidental changes to objects can be avoided.
- Transform2D::inv() does not modify the object, so it is suggested to be declared as `const` to avoid accidental modification.
- Transform2D::operator*=() needs to modify the object calling this operator, so it cannot be declared as `const`.