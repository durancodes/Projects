from random import shuffle

def shuffling (mylist):
    shuffle(mylist)
    return mylist

def player_guess():
    guess=''
    while guess not in ['0','1','2']:
        guess=input("Pick 0 , 1 , or 2 :")
    return int(guess)

def check_guess(mylist,guess):
    if mylist[guess]=='O':
        print("Correct !")
    else:
        print("Incorrect!")

ball=['','O','']

result=shuffling(ball)

guess=player_guess()

check_guess(result,guess)
