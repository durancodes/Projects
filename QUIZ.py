print("Welcome to Duran's Quiz Mania ! \nGive Answers In Terms Of Integers (eg: For Option 1 , Enter 1 . For Option 2 , Enter 2 .)\n")


def resume():
    n=input("Would you like to play the game again?(y/n)\n")
    if n =='y':
        main()
    else:
        quit()

def main():
    score=0
    print("\n")
    print("Let's start with Question NO 1: \n")

    #Question 1
    print("What is CPU stands for? \n1.Central Proceeding Unit\n2.Central Processor Unit\n3.Circle Processor Unit\n4. Central Package Unit")

    print("\n")
    answer=int(input("Enter Your Choice:"))

    if answer==2:
        print("Correct!")
        score+=1
    else:
        print("Wrong Answer :(")

    #Question 2
    print("Who is the Father of the Computer?\n1.Charles Babbage\n2.Thomas Edison\n3.Albert Einstein\n4.Isaac Newton")

    print("\n")
    answer=int(input("Enter Your Choice:"))

    if answer==1:
        print("Correct!")
        score+=1
    else:
        print("Wrong Answer :(")

    #Question 3
    print("In the virtual world, WWW stands for?\n1.World Without Windows\n2.World Wide Web\n3.World Wide Web Applications\n4.World Wide Warehouse")

    print("\n")
    answer=int(input("Enter Your Choice:"))

    if answer==2:
        print("Correct!")
        score+=1
    else:
        print("Wrong Answer :(")

    #Question 4
    print("Which one of the following is not an Operating System (OS)?\n1.Windows 10\n2.Linux\n3.DOS\n4.MS Excel")

    print("\n")
    answer=int(input("Enter Your Choice:"))

    if answer==4:
        print("Correct!")
        score+=1
    else:
        print("Wrong Answer :(")

    #Question 5
    print("What do you need to use to connect to the internet?\n1.Mouse\n2.Modem\n3.CPU\n4.Keyboard")

    print("\n")
    answer=int(input("Enter Your Choice:"))

    if answer==2:
        print("Correct!")
        score+=1
    else:
        print("Wrong Answer :(")

    print("You Have Scored:",score)

    resume()

main()