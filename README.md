# IVBOY - A Powerful FORTH-like Programable Scientific RPN Calculator for the Arduboy

  This software is covered by the 3-clause BSD license.
  See also: https://github.com/zooxo/ivboy

```
  ____________________

    PREAMBLE
  ____________________

  IVBOY is a port if IVEE respectively IVTINY (see https://github.com/zooxo/ivt)
  to the ARDUBOY hardware. Unfortunately the Arduboy has 6 keys only. So a good
  performance as pocket calculator depends on how to use the keys optimally.
  As the executable program has approximately 15 kilobytes (of possible 28)
  there is enough room to expand (ie complex numbers, matrices, graphics).

  So far the upper half of the screen shows always the top of the stack while
  the lower part gives hints to enter commands. Normally you are always in some
  kind of command mode and select one (of 111 possible) functions with the
  cursor keys and the execution key (B). Additionally you are able to reach
  important or user defined commands with the (shifting) key A and the cursor
  keys in a "fast" way (total 32 functions).
  
  ____________________

    COMMANDS
  ____________________

  Important commands: MNU 1 2 3 4 5 6 7 8 9 0 . DROP EE NEG DUP
  Basic operations:   + - / *
  Conditions:         = > < <>
  Other commands (alphabetic order):
    % %CH ABS ACOS ACSH ASIN ASNH ATAN ATNH BATT BEG C*F CA CM* COS COSH DEG
    ELSE EXP FRAC H* HMS* IF INT INV KG* KM* L* LN LN! LOCK LOG LR M*FT ND OVER
    P*R PC PI PICK POW PRG PW10 PV QE R*P RCL ROT S+ SCLR SIN SINH SOLV SQRT
    STAT STO SWAP TAN TANH THEN UNTL USR $A ... $Z

  For an explanation of most commands see: https://github.com/zooxo/ivt

  ____________________

    COMMAND MODE
  ____________________

  After switching on you are in the command mode. While the upper half of the
  screen shows the top of the stack, the lower part shows the selected command.
  On the left side you see the command number (one figure upon the other), the
  command name (maximum of 4 letters) and on the right side the name of the
  previous and next command.
  Select the next command with the cursor down key (and vice versa). To leap
  10 commands forward press the right key (and vice versa).
  To enter the selected command press the ENTER key (B).

  After the first 16 essential commands (figures, basic operations) all further
  commands are sorted alphabetically.
  After 85 commands the (26) programmed user commands ($A ... $Z) can be
  executed.

  But there is a much faster way to execute (16) important commands and even
  (16) user definable commands by pressing the SHIFT key (A).
  While pressing (and holding) A the lower screen shows a "star menu" with 4
  options corresponding to the four cursor keys. After typing the appropriate
  cursor key another "star menu" opens and gives the opportunity to execute one
  (of four) commands with the cursor keys.
  This procedure requires two fingers (hands), but by using the command LOCK one
  finger selection is possible.

  Last but not least you can change the "star menu" of important commands to a
  "star menu" of user defined commands (USER MENU). Enter the number of the
  desired command and the desired position in the user menu to the stack and
  execute the command USR.
  The position pattern of the "star menu" is:              4567
                                                        0123  890a
                                                           bcde

  ____________________

    PROGRAMMING
  ____________________

  Execute one (of maximal 26) user defined programs by selecting the appropriate
  command $A ... $Z.
  To edit a program enter the program number (ie 1 for $A) to the stack and
  execute PRG.
  Browse the program steps with cursor up/down and insert/delete a program step
  with the left/right cursor key. Please note that it is also possible to enter
  a desired command via the user menu.
  Leave and save the program editing with pressing A.

  ____________________

    SPECIAL KEYS
  ____________________

  Set the brightness (6 levels) by pressing (and holding) the left cursor key
  and pressing the upper (brighter) or lower cursor key (darker).
  Goto sleep mode with pressing (and holding) the left cursor key and pressing
  the right kursor key. To wake up the IVBOY press key A.
  Interrupt (endless) loops by pressing cursor up and down key simultaneousely.
  ____________________

```
