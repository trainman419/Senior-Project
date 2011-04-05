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
struct heading compass();

/* initialize compass */
void compass_init();

