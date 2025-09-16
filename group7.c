//Omer Ben Avi 208956342
//Tamir Basson 318918745
//Elay Ardity 208785634
//Yuval Marmur 209191568
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define delim "\t (),\n"
#define MAXCOSIZE 100
#define BMP "fishpool.bmp"
#define BMPCPY "fishpool-copy.bmp"
#define BMPFISH "fishpool-fishing.bmp"
#define TXT "pools.txt"
#define BEST_TXT "best-route.txt"

typedef struct { //pixel color
	int r;
	int g;
	int b;
} color_t;

typedef struct { //pixel's coordinates
	int x;
	int y;
}co_t;

typedef struct pixel { //list of pixels
	co_t p;
	struct pixel* next;
}pix_t;

typedef struct pool { //pools' list extructed of bmp
	int size; //the number of pixels that combine the pool
	co_t poolCenter;
	pix_t* pArr; //list of pool pixels
	struct pool* next;
}poolList_t;

typedef struct color {
	char R;
	char G;
	char B;
}color;

typedef struct tflist {//pools list read from txt file
	co_t p1, p2;
	int size;
	struct tflist* next;
}textfile_t;

typedef struct route_s { //route
	textfile_t* pool;
	int oil;
	int count;
	struct route_s* next;
}route_t;

typedef struct  poolfor3 {  //struct for function 3 (tree/arrs)
	int size;
	co_t poolCenter;
	struct poolfor3* n1;  //closest pool
	float d1;             // distance to closest pool
	struct poolfor3* n2;   //2nd close pool
	float d2;             // distance
	int visit;           //for building tree
	float oil;
	float time;
	int index1;          //index of closest pool in the arr
	int index2;           //index 2nd close pool
}poolList3_t;

typedef struct fishing {     //struct for 5 
	int size;
	co_t poolCenter;
	int visit;
	struct fishing* next;
}fish_t;

//function for 1 2 by Yuval
void option1();											//find pools in bmp and write finding to a text-file
void option2(char);
unsigned char** readBMP(unsigned int* width, unsigned int* hight);			//read bmp file. 
unsigned char checkPool(FILE*);
poolList_t* buildPoolList(unsigned char** matrix, unsigned int wid, unsigned int high);
pix_t* buildPixelsArr(unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int, pix_t*);
pix_t* addpixeltolist(unsigned int x, unsigned int y, pix_t* head, unsigned char** matrix);
int getPixelCount(pix_t*);
co_t get_poolCenter(pix_t*);
co_t getPoolCenter(poolList_t* pools_list);
void freeMat(unsigned char** blue, unsigned int width, unsigned hight);
void printProperties(poolList_t*);
int getPoolsCount(poolList_t*);
void writeFilePoolProp(poolList_t*, char*, int wid, int high);
void freePoolList(poolList_t*);
void freePixelList(pix_t* pixel);											// recortion free functions.
char checkPool(FILE*);
co_t check_max(pix_t*);
textfile_t* getPoolslist(FILE* source_of_pools);
int poolsinlist(textfile_t* head_of_list);
textfile_t** BubbleSort(textfile_t** Arr, int len);							// sorting linked list 
void printsort(textfile_t**, int);											// printing sorted list
void freeTxTfile(textfile_t*);	//free memory allocated for text file read


//function for 3   by Omer and Tamir
void option3();
co_t createstartpoint(char* str, co_t destination);                       //from the input of the user
co_t createDestination();                                                 //taking the cordinate from poolTXT
float getoil();                                                          //initiating oil for input of user
poolList3_t* createpoolarr(co_t startpoint, co_t destination, int* size);//creating arr of structs with poolcenters (first-startpoint last-destination)
void insertDataToARR(poolList3_t* pa, int size);                         //  //insert data of the closest pools and durations   
poolList3_t* all_paths(poolList3_t* pa, poolList3_t* tree, int i);       //for building the tree from arr of pulls
void printroute(poolList3_t* tree, co_t destination, int i, poolList3_t* printarr, poolList3_t* arrfortext);    //sort for routs from the tree and insert to arr. print routes.
void initialarrs(poolList3_t* printarr, poolList3_t* arrfortext, poolList3_t* tree);                            //initial start point in arrs
void initialdesttoprintarr(poolList3_t* printarr, poolList3_t* tree, int i, co_t destination);                  //function for building the route arr   (initial case)
void initialn1toprintarr(poolList3_t* printarr, poolList3_t* tree, int i);                                       //function for building the route arr (n1 case)
void initialn2toprintarr(poolList3_t* printarr, poolList3_t* tree, int i);                                        //function for building the route arr  (n2 case)
int topointtogo(float oil, co_t currentpoint, co_t destination);                                                // can you go with current oil? yes-1 no-0
float calculatedistance(co_t currentpoint, co_t pointtogo);                                                    //distance between to points
//function for 3c by omer and yuval
void print_BMP_cpy(char** mat, unsigned int* wid, unsigned int* high);                                         //take the mat and translate to bmp after changing
char** change_mat(char** mat, poolList3_t* arrfortext);                                                       //changing the mat by insert the route
void openTxtFile(poolList3_t* arrfortext);                                                                   //creating text file with best route
void freeall(poolList3_t* pa, char* str, int size, poolList3_t* printarr, poolList3_t* arrfortext);           //free all


//function for 4 by Elay
void option4();
int count_points(FILE* f);                                               // count all the stops till the end point
co_t File_to_coordinates(FILE* f, co_t p, int m);                        // according to a line, seek the coordinates from the file and put it in a list
pix_t* points_list(pix_t* route, int points, FILE* f);                   // create a linked list of the stops of the route
double distance_between(pix_t* route, pix_t* temp);                      // calculate the distance between 2 stops
double total_distance(pix_t* route);                                     // calculate the total distance of the route
void distance_to_cost(pix_t* route, int points, int skip, double td);    // calculate the cost of the robot actions
void free_list(pix_t* route);                                            // deallocate memory of linked list

//function for 5 by Tamir and Elay
void option5();
poolList3_t* createpoolarr2(co_t destination, int* size);               //for chaking the num of fish 
co_t createlogisticpoint(poolList3_t* pa, int size);                    // calculate avarage of all pools
int total_fish(poolList3_t* pa, int size);                              //total num of fish
int get_fish(int max_fish);                                            //how many to fishing
int findclosetstpool(poolList3_t* pa, int size, co_t logisticpoint);    //for next pool to fish
float takeFishes(poolList3_t* pa, int size, co_t logisticpoint, int ans, float totaldis, fish_t* f);      //makr arr of pools you visit
//bmp for 5 by Omer
char** change_matfor5(char** mat, fish_t* f);                               //changing mat with route of fishing
void print_BMP_fishing(char** mat, unsigned int* wid, unsigned int* high);  //create bmp file
void free_the_list(fish_t* f, poolList3_t* pa);                         //free all


int main(int argc, char* argv[]) {
	char d = argc > 1 ? argv[1][0] : 'f';

	int choice = 0;

	do {
		printf("--------------------------\nME LAB services\n--------------------------\nMenu:\n");
		printf("1. Scan pools\n2. Print sorted pool list\n3. Select route\n4. Numeric report.\n5. Students addition\n9. Exit.\nEnter choice: ");
		choice = fgetc(stdin) - '0';
		if (choice + '0' != '\n' && fgetc(stdin) != '\n')
			while ((choice = fgetc(stdin)) != '\n');
		printf("\n");
		switch (choice) {
		case 1:
			option1(); break;
		case 2:
			option2(d); break;
		case 3:
			option3();
			break;
		case 4:
			option4();
			break;
		case 5:
			option5();
			break;
		case 9:
			printf("Good bye!\n"); break;
		default:
			printf("Bad input, try again\n\n"); break;
		}
	} while (choice != 9);
	return 0;
}


void option1()
{
	unsigned char** mat = NULL;
	unsigned int wid = 0, high = 0;
	poolList_t* poolList = NULL;
	mat = readBMP(&wid, &high);
	if (!mat) return;										//if matrix wasnt found
	poolList = buildPoolList(mat, wid, high);			//builds linked list of pools
	freeMat(mat, wid, high);	//free matrix memory
	printProperties(poolList);
	printf("Total of %d pools.\n", getPoolsCount(poolList));
	writeFilePoolProp(poolList, TXT, wid, high);
	if (poolList != NULL)
		freePoolList(poolList);	//free pool list and pixel array in it								
}

void option2(char d)
{
	FILE* pools;
	int counter = 0;
	textfile_t* maxsize = NULL, * list = NULL, ** Arr;
	errno_t err;
	if (!(err = fopen_s(&pools, TXT, "rt"))) {
		list = getPoolslist(pools);
		if (list != NULL) {
			if (list != NULL) {
				maxsize = list;
				counter = poolsinlist(maxsize);
				Arr = calloc(counter, sizeof(textfile_t*));
				if (Arr == NULL)return;
				for (int i = 0; i < counter; i++) {
					Arr[i] = maxsize;
					maxsize = maxsize->next;
				}
				Arr = BubbleSort(Arr, counter);
				printsort(Arr, counter);
				free(Arr);
				freeTxTfile(list);
			}
		}
		else printf_s("List is empty\n");
	}
	else printf_s("Problem reading pools.txt file\nList is empty\n");
}

//*read bmp image to a matrix
unsigned char** readBMP(unsigned int* wid, unsigned int* high) {
	unsigned char** blue = NULL; //blue matrix of bmp image
	FILE* bmp_open;
	int padding = 0;
	fopen_s(&bmp_open, BMP, "rb");
	if (!bmp_open)
		printf_s("Error open the fishpool.bmp\n");
	if (bmp_open) {
		fseek(bmp_open, 18, SEEK_SET);							//move to location of hight
		fread(wid, sizeof(unsigned int), 1, bmp_open);
		fseek(bmp_open, 22, SEEK_SET);							//move to location of width
		fread(high, sizeof(unsigned int), 1, bmp_open);
		fseek(bmp_open, 54, SEEK_SET);							//move to content of BMP file
		blue = calloc(*wid + 2, 4);
		if (blue == NULL)return NULL;
		for (unsigned int i = 0; i < *wid + 2; i++) {
			blue[i] = calloc(*high + 2, 4);
			if (blue[i] == NULL)return NULL;
		}
		padding = (!(*wid % 4)) ? 0 : (4 - ((*wid) * 3 % 4));			// if modulo 4 != 0 than padding = equasion
		for (unsigned int i = 1; i <= *high; i++)// create rows of matrix 
		{
			for (unsigned int j = 1; j <= *wid; j++) // create cols of matrix
			{
				blue[j][i] = checkPool(bmp_open); // check if pixel is black			

			}
			fseek(bmp_open, padding, SEEK_CUR);// check if size of matrix modolo 4 if not -make padding
		}
	}
	return blue;
}

unsigned char checkPool(FILE* bmp_open) {// function to check if pool or not
	unsigned char blue = '6', red = '6', green = '6';				// the bmp is processed in gray scale BLUE=GREEN=RED.
	blue = fgetc(bmp_open);
	green = fgetc(bmp_open);
	red = fgetc(bmp_open);
	if (blue == 245 && red == 155 && green == 190)  return 1;
	else return 0;
}

poolList_t* buildPoolList(unsigned char** matrix, unsigned int wid, unsigned int high) {
	poolList_t* head = NULL, * first = NULL, * temp = NULL;
	for (unsigned int i = 0; i < high + 2; i++)
	{
		for (unsigned int j = 0; j < wid + 2; j++)
		{
			if (matrix[j][i] == 1) {
				head = calloc(1, sizeof(poolList_t));
				if (!head) return NULL;
				head->pArr = malloc(sizeof(pix_t));
				head->pArr = buildPixelsArr(matrix, wid, high, i, j, head->pArr);  // switch places
				head->size = getPixelCount(head->pArr);
				if (head->size < 10)
					continue;
				head->poolCenter = get_poolCenter(head->pArr);
				head->next = temp;
				temp = head;
			}
		}
	}
	while (temp != NULL)
	{
		head = temp->next;
		temp->next = first;
		first = temp;
		temp = head;
	}
	return first;
}

pix_t* buildPixelsArr(unsigned char** matrix, unsigned int wid, unsigned int high, unsigned int i, unsigned int j, pix_t* head)// clockwise search//
{		//you are standing in (j,i) spot and searching for pool pixel next to (j,i)
	head->p.x = j;
	head->p.y = i;
	head->next = NULL;
	matrix[j][i] = '2';
	pix_t* tail = head, * main = head, * temp = main;
	while (head != NULL)
	{
		if (matrix[head->p.x + 1][head->p.y] == 1)                                //up
			tail = addpixeltolist(head->p.x + 1, head->p.y, head, matrix);
		else if (matrix[head->p.x][head->p.y - 1] == 1)							//left
			tail = addpixeltolist(head->p.x, head->p.y - 1, head, matrix);
		else if (matrix[head->p.x - 1][head->p.y] == 1)							//down
			tail = addpixeltolist(head->p.x - 1, head->p.y, head, matrix);
		else if (matrix[head->p.x][head->p.y + 1] == 1)							//right
			tail = addpixeltolist(head->p.x, (head->p.y + 1), head, matrix);
		else tail = NULL;															//no input
		if (tail == NULL)head = head->next;
		else {
			while (temp->next != NULL)
				temp = temp->next;
			temp->next = tail;
			temp = main;
		}
	}
	return main;
}

pix_t* addpixeltolist(unsigned int x, unsigned int y, pix_t* head, unsigned char** matrix)
{
	matrix[x][y] = '2';
	pix_t* body = malloc(sizeof(pix_t));
	if (body)
	{
		body->p.x = x;
		body->p.y = y;
		body->next = NULL;
	}
	return body;
}

int getPixelCount(pix_t* pixel) {											// checking how many pixels are in pool
	int pixel_count = 0;
	pix_t* temp = pixel->next;

	while (temp != NULL) {
		pixel_count++;
		temp = temp->next;
	}
	return pixel_count + 1;
}

co_t get_poolCenter(poolList_t* pixel_arr) {
	pix_t* current = pixel_arr;
	co_t pool_center;
	int Y_max = current->p.y, Y_min = current->p.y, X_max = current->p.x, X_min = current->p.x;
	while (current != NULL) {
		if (Y_max < current->p.y)
			Y_max = current->p.y;
		if (Y_min > current->p.y)
			Y_min = current->p.y;
		if (X_max < current->p.x)
			X_max = current->p.x;
		if (X_min > current->p.x)
			X_min = current->p.x;
		current = current->next;
	}
	pool_center.x = (X_max - X_min) / 2 + X_min, pool_center.y = (Y_max - Y_min) / 2 + Y_min;;
	return pool_center;
}

void freeMat(unsigned char** blue, unsigned int width, unsigned hight)
{
	int i = width + 2;
	while (i--) free(blue[i]);
	free(blue);
}

void printProperties(poolList_t* first)
{
	if (first != NULL)
	{
		printf_s("Coordinate x1,y1 of the first discoverd pool (%d,%d)\n", first->poolCenter.x, first->poolCenter.y);
		printf_s("Size %d\n", first->size);
	}
}

int getPoolsCount(poolList_t* pool) {										// cheking how many pools in total
	int pool_count = 0;
	while (pool != NULL) {
		pool_count++;
		pool = pool->next;
	}
	return pool_count;
}

void writeFilePoolProp(poolList_t* pool, char* filename, int wid, int high)
{
	FILE* pools;
	fopen_s(&pools, TXT, "wt");
	if (!pools) return;
	fprintf_s(pools, "Image size (%dx%d)\n", wid, high);

	fprintf_s(pools, "Pool Center	Size\n===========	====\n");

	while (pool != NULL) {
		fprintf_s(pools, "(%d,%d)   	%d\n", pool->poolCenter.x, pool->poolCenter.y, pool->size);
		pool = pool->next;

	}
	fclose(pools);
}

void freePoolList(poolList_t* poolList)
{
	freePixelList(poolList->pArr);
	if (poolList->next != NULL)
		freePoolList(poolList->next);
	free(poolList);
}

void freePixelList(pix_t* pixel)
{
	if (pixel->next != NULL)
		freePixelList(pixel->next);
	free(pixel);
}

textfile_t* getPoolslist(FILE* source_of_pools)
{
	char* token = NULL;
	char readline[49]; //delimiters = "\t (),/n";
	int  eof = 0, empty_list = 0;
	textfile_t* pool = NULL, * temp = NULL;

	{
		for (int i = 0; i < 3; i++) {
			fgets(readline, 48, source_of_pools);
		}
		while (eof = fgets(readline, 48, source_of_pools) != 0) {							//stop when file ends
			temp = malloc(sizeof(textfile_t));
			if (temp) {
				temp->p1.x = atoi(strtok_s(readline, delim, &token));					// extracting information from line.
				temp->p1.y = atoi(strtok_s(token, delim, &token));
				temp->size = atoi(strtok_s(token, delim, &token));
				temp->next = pool;														//pushing to end of list
				pool = temp;																//counting number of pools in file.
				empty_list++;
			}
		}
	}
	fclose(source_of_pools);
	return pool;
}

int poolsinlist(textfile_t* head_of_list)
{
	int sum = 0;
	if (head_of_list != NULL)
		if (head_of_list->next != NULL)
			sum = poolsinlist(head_of_list->next);
	return sum + 1;
}

textfile_t** BubbleSort(textfile_t** Arr, int len) {					//sorting the list in an array
	textfile_t** array = Arr;
	textfile_t* temp = NULL;
	int i, j;
	for (i = 0; i < len - 1; i++) {									//sorting method is bubble sort
		for (j = 0; j < len - 1 - i; j++) {
			if (array[j]->size < array[j + 1]->size) {
				temp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = temp;
			}
		}
	}
	return array;
}

void printsort(textfile_t** Arr, int counter) {						//printing sorted pools list.
	printf_s("Sorted pools by size:\n");
	printf_s("Coordinate      Size\n");					//two rows of head line
	printf_s("==========      ====\n");
	for (int i = 0; i < counter; i++) {								//printing till end of file.
		printf_s("(%3d,%3d)   \t%d\n", Arr[i]->p1.x, Arr[i]->p1.y, Arr[i]->size);
	}
}

void freeTxTfile(textfile_t* textPtr) {
	if (textPtr->next != NULL)
		freeTxTfile(textPtr->next);
	free(textPtr);
}


void option3()
{
	unsigned char** mat = NULL;
	co_t startpoint, destination;
	float oil;
	char* str;
	poolList3_t* tree = NULL; poolList3_t* poolarr = NULL;
	int size = 0, width = 0, height = 0;
	tree = malloc(sizeof(poolList3_t));       //malloc to the tree
	str = malloc(sizeof(char) * MAXCOSIZE); //x,y in our format needs 7 char +/0
	destination = createDestination();
	if (destination.x == -1)            //case the file opening failed
		return;
	startpoint = createstartpoint(str, destination);
	oil = getoil();
	poolarr = createpoolarr(startpoint, destination, &size);
	if (poolarr == NULL) {
		return;
	}
	insertDataToARR(poolarr, size);
	tree = all_paths(poolarr, tree, 0);
	tree->oil = oil;
	tree->time = 0;
	poolList3_t* printarr;
	printarr = (poolList3_t*)malloc(size * sizeof(poolList3_t));
	poolList3_t* arrfortext;
	arrfortext = (poolList3_t*)malloc(size * sizeof(poolList3_t));
	initialarrs(printarr, arrfortext, tree);
	printroute(tree, destination, 0, printarr, arrfortext);
	if (arrfortext[1].size > -1)        //for safe
	{
		unsigned char** mat = NULL;
		mat = readBMP(&width, &height);											//creating new mat for BMP copy
		mat = change_mat(mat, arrfortext);						//changing matrix values according to best route
		print_BMP_cpy(mat, &width, &height);                   //printing BMP copy
		freeMat(mat, width, height);
	}
	free(tree);
	freeall(poolarr, str, size, printarr, arrfortext);
}

co_t createDestination()
{
	FILE* myFile = NULL;
	co_t destination = { -1,-1 };
	fopen_s(&myFile, "pools.txt", "rt");
	if (!myFile) {
		printf_s("Error open the pools.txt\n");
		return destination;
	}
	fseek(myFile, 12, SEEK_SET);   //making the way to relevant numbers for destination
	fscanf_s(myFile, "%d", &destination.x);
	fseek(myFile, 1, SEEK_CUR);
	fscanf_s(myFile, "%d", &destination.y);
	fclose(myFile);
	return destination;
}

float getoil() {
	int b;
	float oil;
	do {
		char str[12];
		printf_s("Please enter valid oil supply in range 1-1000\n");
		fgets(str, 12, stdin);            //taking thr string that the user give
		b = sscanf(str, "%f", &oil);   // if read a number b=1 
		if (b == 1) {
			oil = atof(str);
			if (!(1 <= oil && oil <= 1000)) {  //not in range try again
				b = 2;
			}
		}
	} while (b != 1);
	return oil;
}

co_t createstartpoint(char* str, co_t destination) {
	int a;
	co_t startpoint = { -1,-1 };
	do {
		printf_s("Please Enter valid x,y start coordinate, bmp width is %d and height is %d\n", destination.x, destination.y);
		fgets(str, 100, stdin);                                     //taking thr string that the user give
		a = sscanf_s(str, "%d,%d", &startpoint.x, &startpoint.y); //if succes to read a=2
		if (a == 2) {
			if (destination.x < startpoint.x || destination.y < startpoint.y) {//chek if in range of bmp size

				a = 3;
			}
		}
	} while (a != 2);
	return startpoint;
}

poolList3_t* createpoolarr(co_t startpoint, co_t destination, int* size) {       //create pool array through POOL.TXT
	char line[256]; //MAX LINE
	int a, b, f;
	poolList3_t* pa;
	int  count = 0, i, lines = 0;
	FILE* myFile = NULL;
	char c, p;
	char ans[2];
	fopen_s(&myFile, "pools.txt", "rt");
	if (!myFile) {
		printf_s("Error open the pools.txt\n");
		return NULL;
	}
	while (fgets(line, 256, myFile)) {                                       //count the number of lines
		lines++;
		if (line[0] == '\n') {                                                    //if there is only a /n do'nt count
			lines = lines - 1;
		}
	}
	const int n = lines - 1;
	*size = n;                                                              //number of lines equal to the number of pools
	pa = malloc(n * sizeof(poolList3_t));
	pa[0].poolCenter.x = startpoint.x;
	pa[0].poolCenter.y = startpoint.y;
	pa[0].size = 0;
	fseek(myFile, 0, SEEK_SET);                                      // Back to start of the txt file and get the not relavent lines
	fgets(line, 256, myFile);
	fgets(line, 256, myFile);                                         // Get the first 3 lines no relevant
	fgets(line, 256, myFile);
	for (i = 1; i < n - 1; i++) {
		fseek(myFile, 1, SEEK_CUR);                                //initiate DATA
		a = fscanf_s(myFile, "%d", &pa[i].poolCenter.x);
		fseek(myFile, 1, SEEK_CUR);
		b = fscanf_s(myFile, "%d", &pa[i].poolCenter.y);
		fseek(myFile, 5, SEEK_CUR);
		f = fscanf_s(myFile, "%d", &pa[i].size);
		fseek(myFile, 2, SEEK_CUR);
		if (a + b + f != 3) {
			printf_s("File is'nt in a correct format.\n");
			return NULL;
		}
	}
	fclose(myFile);
	pa[n - 1].poolCenter.x = destination.x;
	pa[n - 1].poolCenter.y = destination.y;
	pa[n - 1].size = 0;
	return pa;                                                  //return pool array
}

void insertDataToARR(poolList3_t* pa, int size) {
	for (int i = 0; i < size - 1; i++) {
		pa[i].visit = 0;
		float d1 = 800, d2 = 800;
		int  index1 = -1, index2 = -1;
		for (int j = 1; j < size - 1; j++) {                    //go through all the array
			if (i != j) {
				if (sqrt(pow((double)pa[i].poolCenter.x - (double)pa[j].poolCenter.x, 2) + pow((double)pa[i].poolCenter.y - (double)pa[j].poolCenter.y, 2)) < d1)
				{      //if distance is lower then current distance, change the min distance to the current distance.
					d1 = sqrt(pow((double)pa[i].poolCenter.x - (double)pa[j].poolCenter.x, 2) + pow((double)pa[i].poolCenter.y - (double)pa[j].poolCenter.y, 2));
					index1 = j;    //take the index of the first pool
				}
			}
		}
		pa[i].d1 = d1;       //initiate the value
		pa[i].n1 = &pa[index1];
		pa[i].index1 = index1;
		for (int j = 1; j < size - 1; j++)           //go through all the array
		{
			if (i != j && j != index1) {
				if (sqrt(pow((double)pa[i].poolCenter.x - (double)pa[j].poolCenter.x, 2) + pow((double)pa[i].poolCenter.y - (double)pa[j].poolCenter.y, 2)) < d2)
					//if second distance is lower then current distance, change the min distance to the current distance.
					d2 = sqrt(pow((double)pa[i].poolCenter.x - (double)pa[j].poolCenter.x, 2) + pow((double)pa[i].poolCenter.y - (double)pa[j].poolCenter.y, 2));
				index2 = j;    //take the index of the second closest pool

			}
		}
		pa[i].d2 = d2;            //initiate the value
		pa[i].n2 = &pa[index2];
		pa[i].index2 = index2;
	}
}

poolList3_t* all_paths(poolList3_t* pa, poolList3_t* tree, int i)
{
	int temp = pa[i].visit;
	poolList3_t* hold;
	pa[i].visit = 1; // Mark as visited.
	//go left to closest pool if exist and not visited
	if (pa[i].index1 == -1 || pa[i].n1->visit == 1)
		tree->n1 = NULL;
	else {
		hold = malloc(sizeof(poolList3_t));
		tree->n1 = all_paths(pa, hold, pa[i].index1);
	}
	//go right to 2nd closest pool if exist and not visited
	if (pa[i].index2 == -1 || pa[i].n2->visit == 1)
		tree->n2 = NULL;
	else {
		hold = malloc(sizeof(poolList3_t));
		tree->n2 = all_paths(pa, hold, pa[i].index2);
	}
	pa[i].visit = temp;//make the pool not visited for new optionals paths 
	//initial the node with data about pool
	tree->d1 = pa[i].d1;
	tree->d2 = pa[i].d2;
	tree->index1 = pa[i].index1;
	tree->index2 = pa[i].index2;
	tree->poolCenter.x = pa[i].poolCenter.x;
	tree->poolCenter.y = pa[i].poolCenter.y;
	tree->size = pa[i].size;
	return tree;
}

void printroute(poolList3_t* tree, co_t destination, int i, poolList3_t* printarr, poolList3_t* arrfortext)
{
	static int time = -1;   //for the best route
	if (topointtogo(printarr[i].oil, printarr[i].poolCenter, destination) == 1)  //check if possible to reach destination
	{
		initialdesttoprintarr(printarr, tree, i, destination);  //if yes insert to route arr
		if (time > printarr[i + 1].time || time == -1)         //update the time and print if its the best or first route 
		{
			time = printarr[i + 1].time;
			for (int j = 0; j < i + 1; j++)
				printf_s("Time=%.2f (%d,%d) oil=%.2f -> ", printarr[j].time, printarr[j].poolCenter.x, printarr[j].poolCenter.y, printarr[j].oil);
			printf_s("Time=%.2f (%d,%d) oil=%.2f\n", printarr[i + 1].time, printarr[i + 1].poolCenter.x, printarr[i + 1].poolCenter.y, printarr[i + 1].oil);
			for (int j = 0; j < i + 2; j++)
			{
				arrfortext[j].poolCenter = printarr[j].poolCenter;
				arrfortext[j].size = printarr[j].size;
			}
			if (i == 0)  //direct from start point to dest case
			{
				openTxtFile(arrfortext);
				time = -1;
			}
		}
		return;
	}
	if (tree->n1 != NULL) //if not reach dest try closest pool for the route
	{
		if (topointtogo(printarr[i].oil, printarr[i].poolCenter, tree->n1->poolCenter) == 1) // if make it to the poolcenter update the route arr+time+oil
		{
			initialn1toprintarr(printarr, tree, i);
			printroute(tree->n1, destination, i + 1, printarr, arrfortext);
		}
		if (tree->n2 != NULL)//try second close pool
		{
			if (topointtogo(printarr[i].oil, printarr[i].poolCenter, tree->n2->poolCenter) == 1)
			{
				initialn2toprintarr(printarr, tree, i);
				printroute(tree->n2, destination, i + 1, printarr, arrfortext);
			}
		}
	}
	if (i == 0 && time == -1)   //not make it to dest with all options
	{
		printf_s("Sorry, could not reach destination with these inputs\n\n");
		return;
	}
	else if (i == 0) //make it to dest- create txtfile of best route with arr data
	{
		openTxtFile(arrfortext);
		time = -1;
	}
}

void initialarrs(poolList3_t* printarr, poolList3_t* arrfortext, poolList3_t* tree) //when start the put start point in first place in arr
{
	arrfortext[0].visit = 1;
	printarr[0].oil = tree->oil;
	printarr[0].time = tree->time;
	printarr[0].poolCenter = tree->poolCenter;
	printarr[0].size = tree->size;
}

void initialdesttoprintarr(poolList3_t* printarr, poolList3_t* tree, int i, co_t destination) // put dest point in last place in arr
{
	printarr[i + 1].poolCenter = destination;
	printarr[i + 1].time = printarr[i].time + 5 * calculatedistance(printarr[i].poolCenter, destination);
	printarr[i + 1].size = 0;
	printarr[i + 1].oil = printarr[i].oil - 0.2 * calculatedistance(printarr[i].poolCenter, destination);
}

void initialn1toprintarr(poolList3_t* printarr, poolList3_t* tree, int i)    // if go to n1
{
	printarr[i + 1].poolCenter = tree->n1->poolCenter;
	printarr[i + 1].size = tree->n1->size;
	printarr[i + 1].time = printarr[i].time + 5 * calculatedistance(printarr[i].poolCenter, printarr[i + 1].poolCenter) + printarr[i + 1].size;
	printarr[i + 1].oil = printarr[i].oil - 0.2 * calculatedistance(printarr[i].poolCenter, printarr[i + 1].poolCenter) + 0.2 * printarr[i + 1].size;
}

void initialn2toprintarr(poolList3_t* printarr, poolList3_t* tree, int i)  //if go to n2
{
	printarr[i + 1].poolCenter = tree->n2->poolCenter;
	printarr[i + 1].size = tree->n2->size;
	printarr[i + 1].time = printarr[i].time + 5 * calculatedistance(printarr[i].poolCenter, printarr[i + 1].poolCenter) + printarr[i + 1].size;
	printarr[i + 1].oil = printarr[i].oil - 0.2 * calculatedistance(printarr[i].poolCenter, printarr[i + 1].poolCenter) + 0.2 * printarr[i + 1].size;
}

int topointtogo(float oil, co_t currentpoint, co_t destination) {////if you can go then 1 else 0
	float d = calculatedistance(currentpoint, destination); //Distance
	if (oil * 5 >= d) {
		return 1;
	}
	return 0;
}

float calculatedistance(co_t currentpoint, co_t pointtogo) {  //distance between to cordinate
	float d = sqrt((double)pow((double)pointtogo.x - (double)currentpoint.x, 2) + pow((double)pointtogo.y - (double)currentpoint.y, 2)); //Duration
	return d;
}

void openTxtFile(poolList3_t* arrfortext) {//create text from route arr
	FILE* fp;
	fopen_s(&fp, BEST_TXT, "w");
	if (fp == NULL)
	{
		printf_s("Problem with file best-route.txt, or it might be empty.");
		return;
	}
	fprintf_s(fp, "Best Route	Size\n");
	int i = 0;
	while (arrfortext[i].size > -1)
	{
		fprintf_s(fp, "(%3d,%3d)	%d\n", arrfortext[i].poolCenter.x, arrfortext[i].poolCenter.y, arrfortext[i].size);
		i++;
	}
	printf_s("New best-route.txt file was created\n");
	fclose(fp);
}

char** change_mat(char** mat, poolList3_t* arrfortext) {
	int x1 = 0, y1 = 0, i = 0, x2 = 0, y2 = 0, deltax, deltay;
	while (arrfortext[i + 1].size > -1) { //there is poolcenter in the next place
		x1 = arrfortext[i].poolCenter.x;
		y1 = arrfortext[i].poolCenter.y;
		x2 = arrfortext[i + 1].poolCenter.x;
		y2 = arrfortext[i + 1].poolCenter.y;
		deltax = abs(x2 - x1);
		deltay = abs(y2 - y1);
		if (deltax > deltay)  //chack the correct way to calculate the line function (2cases)
		{                     //3 diffrent way for evrey case to line function depend the position of the one pool center withe the second  
			if ((x2 - x1) >= 0)
				for (int x = 0; x < deltax; x++)
					mat[x1 + x][(int)(round((double)(y2 - y1) / (double)(x2 - x1) * ((double)x1 + (double)x - (double)x1) + (double)y1))] = 'b';
			else if ((x1 - x2) >= 0 && (y1 - y2) >= 0)
				for (int x = 0; x < deltax; x++)
					mat[x2 + x][(int)(round((double)(y1 - y2) / (double)(x1 - x2) * ((double)x2 + (double)x - (double)x2) + (double)y2))] = 'b';
			else
				for (int x = 0; x < deltax; x++)
					mat[x1 - x][(int)(round((double)(y2 - y1) / (double)(x1 - x2) * ((double)x1 + (double)x - (double)x1) + (double)y1))] = 'b';
		}
		else
		{
			if ((y2 - y1) > 0)
				for (int y = 0; y < deltay; y++)
					mat[(int)(round(((double)(x2 - x1) / (double)(y2 - y1)) * ((double)y1 + (double)y - (double)y1) + (double)x1))][y1 + y] = 'b';
			else if ((x1 - x2) >= 0 && (y1 - y2) >= 0)
				for (int y = 0; y < deltay; y++)
					mat[(int)(round(((double)(x1 - x2) / (double)(y1 - y2)) * ((double)y2 + (double)y - (double)y2) + (double)x2))][y2 + y] = 'b';
			else
				for (int y = 0; y < deltay; y++)
					mat[(int)(round((double)(x2 - x1) / (double)(y1 - y2) * ((double)y1 + (double)y - (double)y1) + (double)x1))][y1 - y] = 'b';
		}
		i++;
	} i = 0;

	while (arrfortext[i + 1].size > -1)
	{
		mat[arrfortext[i + 1].poolCenter.x][arrfortext[i + 1].poolCenter.y] = 'g';// for all the pool centers
		i++;
	}
	mat[arrfortext[0].poolCenter.x][arrfortext[0].poolCenter.y] = 'r';// for start point

	return mat;
}

void print_BMP_cpy(char** mat, unsigned int* wid, unsigned int* high) {
	char bmp_header[55];																//assigning identifires 
	int padding = 0;
	FILE* BMP_cpy, * bmp_open;
	color red = { 0,0,255 }, green = { 0,255,0 }, blue = { 255,0,0 }, paint;
	color pool = { 245,190,155 }, white = { 255,255,255 };
	padding = (!(*wid % 4)) ? 0 : (4 - ((*wid) * 3 % 4));									// checking if needed to take padding in consideration
	fopen_s(&bmp_open, BMP, "rb");																// open original BMP  read BMP 
	if (!bmp_open)return;																		// if original bmp didnt open.
	fread(bmp_header, 54, 1, bmp_open);															// copy original bmp header.
	fclose(bmp_open);																			// close original bmp
	fopen_s(&BMP_cpy, BMPCPY, "wb+");
	if (!BMP_cpy) return;																		// if binary file havent open.
	for (int i = 0; i < sizeof(bmp_header) - 1; i++)
		fputc(bmp_header[i], BMP_cpy);															// printing header bmp file.
	for (unsigned int i = 1; i <= *high; i++)													//moving along the rows of matrix
	{
		for (unsigned int j = 1; j <= *wid; j++)												//moving along the collums of matrix
		{
			if (mat[j][i] == 'b')paint = blue;													// checking what's inside the matrix 
			else if (mat[j][i] == 'r')paint = red;												//and printing accordinglly
			else if (mat[j][i] == 'g')paint = green;
			else if (mat[j][i] == 1)paint = pool;
			else paint = white;
			fputc(paint.R, BMP_cpy);
			fputc(paint.G, BMP_cpy);
			fputc(paint.B, BMP_cpy);
		}
		if (padding)fwrite("\0", sizeof(char), padding, BMP_cpy);							//add padding info if needed.
	}
	printf_s("A BMP file fishpool-copy.bmp was created\n\n");

	fclose(BMP_cpy);
}

void freeall(poolList3_t* pa, char* str, int size, poolList3_t* printarr, poolList3_t* arrfortext) {//free all
	free(pa);
	free(printarr);
	free(arrfortext);
	free(str);
}


void option4()
{
	unsigned int  a;
	FILE* f;
	double skip;
	int b, c, d;
	a = fopen_s(&f, BEST_TXT, "rt");
	if (a != 0)
	{
		printf_s("Problem with file best-route.txt, or it might be empty.\n");
		return;
	}

	int points = count_points(f);
	pix_t* route = NULL;
	route = points_list(route, points, f);
	double distance = total_distance(route);
	printf_s("Please enter a positive intger as distance display interval:\n");
	do {
		b = 1;
		d = scanf_s("%lf", &skip);
		while ((c = getchar()) != '\n' && c != EOF);
		if (skip - (int)skip != 0 || skip<1 || skip>distance)
		{
			printf_s("Bad input, try again\n");
			b = 2;
		}
	} while (b != 1);

	distance_to_cost(route, points, (int)skip, distance);
	free_list(route);
	return 0;
}

// count all the stops till the end point
int count_points(FILE* f)
{
	char STR[30];
	int  count = 0;
	fgets(STR, sizeof(STR), f);
	while (fgets(STR, sizeof(STR), f))
		count++;
	return count;
}

// according to a line, seek the coordinates from the file and put it in a list
co_t File_to_coordinates(FILE* f, co_t p, int m)
{
	char STR[30];
	fseek(f, 0, SEEK_SET);
	fgets(STR, sizeof(STR), f);
	for (int row = 0; row < m; row++)
		fgets(STR, sizeof(STR), f);
	p.x = atoi(strchr(STR, '(') + 1);
	p.y = atoi(strchr(STR, ',') + 1);
	return p;
}

// create a linked list of the stops of the route
pix_t* points_list(pix_t* route, int points, FILE* f)
{
	pix_t* temp1 = NULL;
	pix_t* temp2 = NULL;
	for (int n = 1; n <= points; n++)
	{
		temp1 = malloc(sizeof(pix_t));
		if (!temp1)
			return 0;
		temp1->p = File_to_coordinates(f, temp1->p, n);
		if (temp2)
			temp2->next = temp1;
		else
			route = temp1;
		temp2 = temp1;
	}
	if (temp2)
		temp2->next = NULL;

	return route;
}

// calculate the distance between 2 points
double distance_between(pix_t* route, pix_t* temp)
{
	return sqrt(pow((route->p.y - temp->p.y), 2) + pow((route->p.x - temp->p.x), 2));
}

// calculate the total distance of the route
double total_distance(pix_t* route)
{
	double d = 0;
	pix_t* temp;
	temp = malloc(sizeof(pix_t));
	if (!temp) return 0;
	temp->p.x = route->p.x;
	temp->p.y = route->p.y;
	route = route->next;
	while (route)
	{
		d = d + distance_between(route, temp);
		temp = route;
		route = route->next;
	}

	return d;
}

// calculate the cost of the robot actions
void distance_to_cost(pix_t* route, int points, int skip, double td)
{
	double cost = 0, d = 0, a = 2.5, dx = 0.1;
	int count = 1, FT = 1, FM = 20;
	printf_s("Distance   Consumption\n========== ===========\n");
	printf_s(" %9.3lf %10.3lf\t\n", d, cost);
	pix_t* temp;
	temp = malloc(sizeof(pix_t));
	if (!temp) return;
	temp->p.x = route->p.x;
	temp->p.y = route->p.y;
	route = route->next;
	d = distance_between(route, temp);
	while (route)
	{
		for (double i = 0.1; i + 0.1 < td; i = i + 0.1)
		{
			if (i > d)
			{
				cost = ((a / (cost + 1) + FM) * dx) + cost;
				temp = route;
				route = route->next;
				d = d + distance_between(route, temp);
			}
			else
				cost = ((a / (cost + 1) + FT) * dx) + cost;
			if (count % (skip * 10) == 0 && i != 0.1)
				printf_s(" %9.3lf %10.3lf\t\n", i, cost);
			count++;

		}
		cost = ((2.5 / (cost + 1) + FT) * 0.1) + cost;
		printf_s(" %9.3lf %10.3lf\n", d, cost);
		temp = route;
		route = route->next;
	}

}

// deallocate memory of linked list
void free_list(pix_t* route)
{
	pix_t* temp = route;
	while (route)
	{
		temp = route->next;
		free(route);
		route = temp;
	}
}


void option5()
{
	unsigned char** mat = NULL;
	co_t startpoint, destination, logisticpoint;
	float oil;
	int max_fish = 0;
	int fish;
	double costfish;
	float totaldis = 0;
	poolList3_t* poolarr = NULL;
	int size = 0, width = 0, height = 0;
	destination = createDestination();
	poolarr = createpoolarr2(destination, &size);
	if (poolarr == 0) {
		return;
	}
	if (size == 0)// no pools case
	{
		printf_s("sorry... there is no pools so there is no fish for you :(\n");
		return;
	}
	insertDataToARR(poolarr, size);
	logisticpoint = createlogisticpoint(poolarr, size);
	max_fish = total_fish(poolarr, size);
	fish = get_fish(max_fish);
	int r = size + 2;
	fish_t* f = malloc((r) * sizeof(fish_t));
	totaldis = takeFishes(poolarr, size, logisticpoint, fish, totaldis, f);
	costfish = fish;
	if (fish >= 100)
		costfish = fish * 0.75;
	printf_s("\n\tYour order is:\n-----------------------------------\nThe price for the fish is: %8.3lf\nThe price for fuel is:   %10.3lf\nThe total price is:   %13.3lf\n-----------------------------------\nThank you for your order!\n", costfish, totaldis * 0.2, (totaldis * 0.2) + costfish);
	void free_the_list(f, poolarr);
}

//read from the pool.txt file and insert the pools to an array
poolList3_t* createpoolarr2(co_t destination, int* size) {
	char line[256];
	int a, b, f;
	poolList3_t* pa;
	int  count = 0, i, lines = 0;
	FILE* myFile = NULL;
	char c, p;
	char ans[2];
	fopen_s(&myFile, "pools.txt", "rt");
	if (!myFile) {
		printf_s("Fail to open file\n");
		return NULL;
	}
	while (fgets(line, 256, myFile)) {
		lines++;
		if (line[0] == '\n') {
			lines = lines - 1;
		}
	}
	const int n = lines - 3;
	*size = n;
	pa = malloc(n * sizeof(poolList3_t));

	fseek(myFile, 0, SEEK_SET); // Back to start of the txt file and get the not relavent lines
	fgets(line, 256, myFile);
	fgets(line, 256, myFile); // Get the first 3 lines no relevant
	fgets(line, 256, myFile);
	for (i = 0; i < n; i++) {
		fseek(myFile, 1, SEEK_CUR);//initiate DATA
		a = fscanf_s(myFile, "%d", &pa[i].poolCenter.x);
		fseek(myFile, 1, SEEK_CUR);
		b = fscanf_s(myFile, "%d", &pa[i].poolCenter.y);
		fseek(myFile, 5, SEEK_CUR);
		f = fscanf_s(myFile, "%d", &pa[i].size);
		fseek(myFile, 2, SEEK_CUR);
		if (a + b + f != 3) {
			printf_s("TXT pools isnt in correct format\n");
			return NULL;
		}
	}
	fclose(myFile);
	return pa;
}

//find the coordinates of the logistic point
co_t createlogisticpoint(poolList3_t* pa, int size) {
	co_t logisticpoint = { -1,-1 };
	int sumx = 0, sumy = 0;
	for (int i = 0; i < size; i++) {
		sumx = sumx + pa[i].poolCenter.x;
		sumy = sumy + pa[i].poolCenter.y;
	}
	logisticpoint.x = (int)((sumx / size) + 0.5);
	logisticpoint.y = (int)((sumy / size) + 0.5);

	return logisticpoint;
}

//calculate how many fish can be sold in one order 
int total_fish(poolList3_t* pa, int size)
{
	int sum = 0;
	for (int i = 0; i < size; i++)
		sum = sum + pa[i].size;
	return sum;
}

//take ordet from the consumer and check the input
int get_fish(int max_fish)
{
	int b;
	int ans;
	int fish;
	do {
		char str[12];
		printf_s("How many fish do you want? 0-%d\nFor at least 100 fish you will get 25%% discount.\n", max_fish);
		fgets(str, 12, stdin);
		b = sscanf(str, "%d", &ans);
		if (b == 1) {
			fish = atoi(str);
			if (!(1 <= fish && fish <= max_fish)) {
				b = 2;
			}
		}
	} while (b != 1);
	return ans;
}

//find the closest pool to an an another point 
int findclosetstpool(poolList3_t* pa, int size, co_t logisticpoint) {
	double min = 8000;
	int index = -1;
	for (int i = 0; i < size; i++) {
		if (logisticpoint.x == pa[i].poolCenter.x && logisticpoint.y == pa[i].poolCenter.y) {
			continue;
		}
		if (pa[i].visit == 0) {


			double dist = calculatedistance(logisticpoint, pa[i].poolCenter);
			if (dist < min) {
				min = dist;
				index = i;//take index of the closest pool
			}
		}
	}
	return index;
}

//calculate the total distance the robot crossed to get all the fish
float takeFishes(poolList3_t* pa, int size, co_t logisticpoint, int ans, float totaldis, fish_t* f) {
	for (int j = 0; j < size + 1; j++) {
		pa[j].visit = 0;
	}
	int fishcount = 0;
	totaldis = 0;
	int oldindex = -1;

	int index = findclosetstpool(pa, size, logisticpoint);//go to the first closestpool
	totaldis += calculatedistance(logisticpoint, pa[index].poolCenter);
	fishcount = fishcount + pa[index].size;//newlist.next=pa[index];
	pa[index].visit = 1;
	f[0].poolCenter = logisticpoint;
	f[1].poolCenter = pa[index].poolCenter;
	f[0].size = 1;
	f[1].size = 1;
	f[2].poolCenter = logisticpoint;
	f[2].size = 0;
	for (int i = 2; fishcount < ans; i++) {
		oldindex = index;
		index = findclosetstpool(pa, size, pa[index].poolCenter);
		if ((pa[index].visit) != 1)
		{
			(totaldis) += calculatedistance(pa[oldindex].poolCenter, pa[index].poolCenter);//todo size dis
			fishcount = fishcount + pa[index].size;
			pa[oldindex].visit = 1;
			f[i].poolCenter = pa[index].poolCenter;
			f[i].size = 1;
			f[i + 1].poolCenter = logisticpoint;
			f[i + 1].size = 0;
		}
	}

	fishcount = ans;
	if (f[1].size > 0)
	{
		unsigned int width = 0, height = 0;
		unsigned char** mat = NULL;
		mat = readBMP(&width, &height);							//creating new mat for BMP copy
		mat = change_matfor5(mat, f);						//changing matrix values according to best route
		print_BMP_fishing(mat, &width, &height);            //printing BMP copy
		freeMat(mat, width, height);
	}
	return totaldis;
}

void print_BMP_fishing(char** mat, unsigned int* wid, unsigned int* high) {
	char bmp_header[55];																//assigning identifires 
	int padding = 0;
	FILE* BMP_fish, * bmp_open;
	color red = { 0,0,255 }, pink = { 255,0,255 }, yellow = { 27,203,239 }, paint;
	color pool = { 245,190,155 }, white = { 255,255,255 };
	padding = (!(*wid % 4)) ? 0 : (4 - ((*wid) * 3 % 4));									// checking if needed to take padding in consideration
	fopen_s(&bmp_open, BMP, "rb");																// open original BMP!! ����� �� ����� ��������� read BMP !!
	if (!bmp_open)return;																		// if original bmp didnt open.
	fread(bmp_header, 54, 1, bmp_open);															// copy original bmp header.
	fclose(bmp_open);																			// close original bmp
	fopen_s(&BMP_fish, BMPFISH, "wb+");
	if (!BMP_fish) return;																		// if binary file havent open.
	for (int i = 0; i < sizeof(bmp_header) - 1; i++)
		fputc(bmp_header[i], BMP_fish);															// printing header bmp file.
	for (unsigned int i = 1; i <= *high; i++)													//moving along the rows of matrix
	{
		for (unsigned int j = 1; j <= *wid; j++)												//moving along the collums of matrix
		{
			if (mat[j][i] == 'y')paint = yellow;
			else if (mat[j][i] == 'r')paint = red;// checking what's inside the matrix and printing accordinglly
			else if (mat[j][i] == 'p')paint = pink;
			else if (mat[j][i] == 1)paint = pool;
			else paint = white;
			fputc(paint.R, BMP_fish);
			fputc(paint.G, BMP_fish);
			fputc(paint.B, BMP_fish);
		}
		if (padding)fwrite("\0", sizeof(char), padding, BMP_fish);							//add padding info if needed.
	}
	printf_s("\nA BMP file fishpool-fishing.bmp was created\n");

	fclose(BMP_fish);
}

char** change_matfor5(char** mat, fish_t* f) {  // same as in 3 but diffrent in the stop case
	int x1 = 0, y1 = 0;
	int x2 = 0, y2 = 0;
	int i = 0;
	int deltax;
	int deltay;
	while (f[i].size != 0) {  //there is next pool in arr
		x1 = f[i].poolCenter.x;
		y1 = f[i].poolCenter.y;
		x2 = f[i + 1].poolCenter.x;
		y2 = f[i + 1].poolCenter.y;
		deltax = abs(x2 - x1);
		deltay = abs(y2 - y1);
		if (deltax > deltay)
		{
			if ((x2 - x1) >= 0)
				for (int x = 0; x < deltax; x++)
					mat[x1 + x][(int)(round((double)(y2 - y1) / (double)(x2 - x1) * ((double)x1 + (double)x - (double)x1) + (double)y1))] = 'y';
			else if ((x1 - x2) >= 0 && (y1 - y2) >= 0)
				for (int x = 0; x < deltax; x++)
					mat[x2 + x][(int)(round((double)(y1 - y2) / (double)(x1 - x2) * ((double)x2 + (double)x - (double)x2) + (double)y2))] = 'y';
			else
				for (int x = 0; x < deltax; x++)
					mat[x1 - x][(int)(round((double)(y2 - y1) / (double)(x1 - x2) * ((double)x1 + (double)x - (double)x1) + (double)y1))] = 'y';
		}
		else
		{
			if ((y2 - y1) > 0)
				for (int y = 0; y < deltay; y++)
					mat[(int)(round(((double)(x2 - x1) / (double)(y2 - y1)) * ((double)y1 + (double)y - (double)y1) + (double)x1))][y1 + y] = 'y';
			else if ((x1 - x2) >= 0 && (y1 - y2) >= 0)
				for (int y = 0; y < deltay; y++)
					mat[(int)(round(((double)(x1 - x2) / (double)(y1 - y2)) * ((double)y2 + (double)y - (double)y2) + (double)x2))][y2 + y] = 'y';
			else
				for (int y = 0; y < deltay; y++)
					mat[(int)(round((double)(x2 - x1) / (double)(y1 - y2) * ((double)y1 + (double)y - (double)y1) + (double)x1))][y1 - y] = 'y';
		}
		i++;
	}  i = 0;
	mat[f[0].poolCenter.x][f[0].poolCenter.y] = 'r';  //logistic point
	i++;
	while (f[i].size != 0) //all the pool center visited for fishing
	{
		mat[f[i].poolCenter.x][f[i].poolCenter.y] = 'p';
		i++;
	}
	return mat;
}

//free linked list from the memory
void free_the_list(fish_t* f, poolList3_t* pa)
{
	free(f);
	free(pa);
}