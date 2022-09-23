/*********************
 *      FUNCTIONS
 *********************/

/*
 * Function:  input_init 
 * --------------------
 * 
 * Initialize the mux driver and set the right configuration.
 * 
 *  Returns: Nothing
 */

void input_init(void);

/*
 * Function:  input_read 
 * --------------------
 * 
 *  Gets the value of the buttons attached to the mux and if the menu button is pushed,
 *  it peforms some special functions such as brightness and volumen set or open the on
 *  game menu.
 * 
 *  Returns: An unsigned interger of 16bit with the status of each button. See the next 
 *  table to know the position of each button.
 *  Bit position of each button
 *  - 0 -> Down     - 2 -> Up
 *  - 1 -> Left     - 3 -> Right
 * 
 *  - 12 -> Select  - 11 -> Menu
 *  - 10 -> Start
 * 
 *  - 9 -> A    - 8 -> B    - 7 -> Y
 *  - 6 -> X    - 5 -> R    - 13 -> L
 * 
 * -----
 * 
 * New bitout
 *  - 0 -> Right     - 2 -> Dowm
 *  - 1 -> Up        - 3 -> Left
 * 
 *  - 4 -> Menu
 *  - 7 -> Select 
 *  - 9 -> Start
 *
 *  - 5 -> A
 *  - 6 -> B
 */

uint16_t input_read(void);