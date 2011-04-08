/* compass.h
 *
 * header file for assembly compass routines.
 *
 * Author: Austin Hendrix
 */

/* heading sturcture */
struct heading
{
	int x;
	int y;
};

/* take compass reading */
void compass();
struct heading compass_poll();

/* initialize compass */
void compass_init();

