///////////////////////////////////////////////////////////////////////////////
//FILE: 'hyper_functions.cpp'
// Contains utilitiy functions for the 'Hyper' class:
//		array sizing
//		writing banners to output
//		writing data to output
//
//030627 Created by Peter H Zipfel
//060512 Updated from F16C for CRUISE, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'hyper5', 'scrn_hyper5', 'plot_hyper5'
//and 'com_hyper5'
// 
//Out to Hyper:: nhyper5, nscrn_hyper5, nplot_hyper5, ncom_hyper5,
//	round3_scrn_count, hyper_scrn_count, round3_plot_count, hyper_plot_count 		 ,
//
//001212 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::sizing_arrays()
{
	const char *key1="empty";
	const char *key2="scrn";
	const char *key3="plot";
	const char *key4="com";
	int round3_full_count=0;
	int hyper_full_count=0;
	int i(0);

	//initialize 'Round3' and 'Hyper' member variables
	round3_scrn_count=0;
	hyper_scrn_count=0;
	round3_plot_count=0;
	hyper_plot_count=0;
	round3_com_count=0;
	hyper_com_count=0;

	//counting in 'round3' array
 	for(i=0;i<NROUND3;i++)
	{
		if(strcmp(round3[i].get_name(),key1))
			round3_full_count++;
		if(strstr(round3[i].get_out(),key2))
			round3_scrn_count++;
		if(strstr(round3[i].get_out(),key3))
			round3_plot_count++;
		if(strstr(round3[i].get_out(),key4))
			round3_com_count++;
	}
	//counting in 'hyper' array
 	for(i=0;i<NHYPER;i++)
	{
		if(strcmp(hyper[i].get_name(),key1))
			hyper_full_count++;
		if(strstr(hyper[i].get_out(),key2))
			hyper_scrn_count++;
		if(strstr(hyper[i].get_out(),key3))
			hyper_plot_count++;
		if(strstr(hyper[i].get_out(),key4))
			hyper_com_count++;
	}
	//output to Hyper::protected
	nhyper5=round3_full_count+hyper_full_count;
	nscrn_hyper5=round3_scrn_count+hyper_scrn_count;
	nplot_hyper5=round3_plot_count+hyper_plot_count;
	ncom_hyper5=round3_com_count+hyper_com_count;
}

///////////////////////////////////////////////////////////////////////////////
//Building 'hyper5' module-array by eliminating empty slots in 'round3' and 'hyper'
//and merging the two arrays 
//
//Output: Hyper::hyper5[] 
//
//001212 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::vehicle_array()
{
	const char *test="empty";
	int i(0);

	//load nonempty slots from round3 array into hyper5 array
	int k=0;
	for(i=0;i<NROUND3;i++)
	{
		if(strcmp(round3[i].get_name(),test))
		{
			hyper5[k]=round3[i];
			k++;
		}
	}	
	//load nonempty slots from hyper array into hyper5 array	
	int m=0;
	for(i=0;i<NHYPER;i++)
	{
		if(strcmp(hyper[i].get_name(),test))
		{
			hyper5[k+m]=hyper[i];
			m++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Building 'scrn_hyper5' module-array from 'hyper5' array by keying on the word 'scrn'
//
//Output: Hyper::scrn_hyper5[] 
//
//001214 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::scrn_array()
{
	int k(0);
	const char *buff;
	const char *key="scrn";

	for(int i=0;i<nhyper5;i++)
	{
		buff=hyper5[i].get_out();
		if(strstr(buff,key))
		{
			scrn_hyper5[k]=hyper5[i];
			k++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Building 'plot_hyper5' module-array from 'hyper5' array by keying on the word 'plot'
//
//Output: Hyper::plot_hyper5[] 
//
//001214 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::plot_array()
{
	int k(0);
	const char *buff;
	const char *key="plot";

	for(int i=0;i<nhyper5;i++)
	{
		buff=hyper5[i].get_out();
		if(strstr(buff,key))
		{
			plot_hyper5[k]=hyper5[i];
			k++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to screen
//
//First label is time-since-launch-of-vehicle 'time', always at round3[0]
// eight accross, unlimited down
// data field will be 15 spaces, total width 120 spaces
// labels longer than 14 characters will be truncated
//Accomodates 3x1 vectors only
//
//010106 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::scrn_banner()
{
	const char *buff1;
	char buff2[15];
	int label_length=14;
	int k(0);

	cout<<"\n Vehicle: HYPER5 "<<'\n'; 

	for(int i=0;i<nscrn_hyper5;i++)
	{
		cout.setf(ios::left);

		buff1=scrn_hyper5[i].get_name();

		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length); //!zi060927
		buff2[14]=0;

		//vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int i=1;i<4;i++)
			{
				
				cout.width(strlen(buff2));
				cout<<buff2;cout.width(15-strlen(buff2));cout<<i;
				k++;
				if(k>7){k=0;cout<<'\n';}
			}
		}
		else
		{
			cout.width(15);
			cout<<buff2;
			k++;
			if(k>7){k=0;cout<<'\n';}
		}
	}
	cout<<"\n\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to 'tabout.asc'
//
//First label is time-since-launch-of-vehicle 'time', always at round3[0]
// eight across, unlimited down
// data field will be 15 spaces, total width 120 spaces
// labels longer than 14 characters will be truncated
//Accommodates 3x1 vectors only
//
//010114 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::tabout_banner(ofstream &ftabout,const char *title)
{
	const char *buff1;
	char buff2[15];
	int label_length=14;
	int k(0);

	ftabout<<"\n"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";
	ftabout<<"\n Vehicle: HYPER5 "<<'\n';
	

	for(int i=0;i<nscrn_hyper5;i++)
	{
		ftabout.setf(ios::left);

		buff1=scrn_hyper5[i].get_name();

		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;

		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int i=1;i<4;i++)
			{				
				ftabout.width(strlen(buff2));
				ftabout<<buff2;ftabout.width(15-strlen(buff2));ftabout<<i;
				k++;
				if(k>7){k=0;ftabout<<'\n';}
			}
		}
		else
		{
			ftabout.width(15);
			ftabout<<buff2;
			k++;
			if(k>7){k=0;ftabout<<'\n';}
		}
	}
	ftabout<<"\n\n";
}
///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each hyper vehicle object
//Assigning initial values to module-variables of 'round3' and 'hyper' arrays
//Reading aero and propulsion decks
//
//The first hyper object 'input.asc' reads until the first 'END' after
// 'HYPER5'. The second hyper object reads until the second 'END', etc until
// the data for all hyper objects are read
//
//Output:	hyper5_name ('Hyper' data member)
//			round3[] data values (Round3 data member)
//			hyper[] data values ('Hyper' data member)
//			event_ptr_list[] ('Event' data members)
//
//Limitation: only real and integer variables can be read 			 
//
//001230 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
//030727 New table look-up scheme, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::vehicle_data(fstream &input)
{
	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	const char *buff=NULL;			//name of module-variable
	double data(0);		//data of module-variable 
	char file_name[CHARN];	//name of data-deck file
	const char *integer=NULL;
	int int_data(0);
	char buff2[CHARN];
	int file_ptr=NULL;
	char name[CHARN];
	Variable *variable=NULL;
	char oper;
	double value(0);
	int e(0);
	const char *watchpoint=NULL;
	int i(0);

	input.getline(hyper5_name,CHARL,'\n');
	//reading data until 'END' is encountered
	do
	{
		//reading variable data into module-variable arrays after discarding comments
		input>>read;
		if(ispunct(read[0]))
		{
			input.getline(line_clear,CHARL,'\n');
		}
		else
		{
			for(i=0;i<NROUND3;i++)
			{
				buff=round3[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=round3[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading integer value
						int_data=(int)data;
						round3[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						round3[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			for(i=0;i<NHYPER;i++)
			{
				buff=hyper[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=hyper[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						hyper[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						hyper[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			//reading aero data from aero-deck file
			if(!strcmp(read,"AERO_DECK")){
				//reading aerodeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,aerotable);
			}

			//reading prop data from prop-deck file
			if(!strcmp(read,"PROP_DECK")){
				//reading propdeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,proptable);
			}

			//reading events into 'Event' pointer array 'event_ptr_list' of size NEVENT
			if(!strcmp(read,"IF"))
			{
				event_total++;
				//reading name of watch variable
				input>>name;
				//determining it's module-variable
				int m;
				for(m=0;m<NROUND3;m++)
				{
					buff=round3[m].get_name();
					if(!strcmp(buff,name)) 
						variable=&round3[m];
				}
				for(m=0;m<NHYPER;m++)
				{
					buff=hyper[m].get_name();
					if(!strcmp(buff,name)) 
						variable=&hyper[m];
				}

				//reading other criteria
				input>>oper;
				input>>value;
				event_ptr_list[e]->set_variable(variable);
				event_ptr_list[e]->set_value(value);
				event_ptr_list[e]->set_operator(oper);
				input.getline(line_clear,CHARL,'\n');

				//acquiring indices and values of event variables
				int el1=0; //element # in round3[]
				int el2=0; //element # in hyper[]
				event_ptr_list[e]->set_round3_size(el1);
				event_ptr_list[e]->set_hyper_size(el2);
				do
				{
					input>>buff2;
					if(ispunct(read[0]))
					{
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						int k;
						for(k=0;k<NROUND3;k++)
						{
							buff=round3[k].get_name();
							if(!strcmp(buff,buff2))
							{
								event_ptr_list[e]->set_round3_index(el1,k);
								input>>data;
								event_ptr_list[e]->set_round3_value(el1,data);
								input.getline(line_clear,CHARL,'\n');
								el1++;
								event_ptr_list[e]->set_round3_size(el1);
								if(el1==NVAR)
								{
									cerr<<"*** Error: Check EVENTS (size of NVAR) *** \n";
									exit(1);
								}									
							}				
						}
						for(k=0;k<NHYPER;k++)
						{
							buff=hyper[k].get_name();
							if(!strcmp(buff,buff2))
							{
								event_ptr_list[e]->set_hyper_index(el2,k);
								input>>data;
								event_ptr_list[e]->set_hyper_value(el2,data);
								input.getline(line_clear,CHARL,'\n');
								el2++;
								event_ptr_list[e]->set_hyper_size(el2);
								if(el2==NVAR)
								{
									cerr<<"*** Error: Check EVENTS (size of NVAR) *** \n";
									exit(1);
								}									
							}				
						}
					}
				}while(strcmp(buff2,"ENDIF"));
				//increment event counter
				e++;	
			} //end of loading events

		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'round3[]' and hyper[] variables 
// that are output to screen  
//
//Output: Hyper::round3_scrn_ind[], hyper_crn_ind[] 
//
//001213 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::scrn_index_arrays()
{
	const char *test="scrn";
	int k=0;
	int i(0);

	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),test))
		{
			round3_scrn_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NHYPER;i++)
	{
		if(strstr(hyper[i].get_out(),test))
		{
			hyper_scrn_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to screen
//
//First label is time-since-launch-of-vehicle 'time', always at round3[0]
//Accomodates real, integers (printed as real) and 3x1 vectors 
//Eight accross, unlimited down
//Data field 15 spaces, total width 120 spaces
//
//010112 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::scrn_data()
{

	int index;
	const char *integer;
	const char *vector;
	Matrix VEC(3,1);
	int k(0);
	int i(0);
	
	cout<<hyper5_name<<'\n';
	cout.setf(ios::left);

	//writing to screen the variables from the 'Round3' class
	for(i=0;i<round3_scrn_count;i++)
	{
		index=round3_scrn_ind[i];
		//checking for integers
		integer=round3[index].get_type();
		vector=round3[index].get_name();
		if(!strcmp(integer,"int"))
		{
			cout.width(15);
			cout<<round3[index].integer();			
			k++; if(k>7){k=0;cout<<'\n';}
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=round3[index].vec();

			cout.width(15);
			cout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;cout<<'\n';}
		}
		else
		{
			//real variables
			cout.width(15);
			cout<<round3[index].real();
			k++; if(k>7){k=0;cout<<'\n';}
		}
	}
	//writing to screen the variables from the 'Hyper' class
	for(i=0;i<hyper_scrn_count;i++)
	{
		index=hyper_scrn_ind[i];
		//checking for integers
		integer=hyper[index].get_type();
		vector=hyper[index].get_name();
		if(!strcmp(integer,"int"))
		{
			cout.width(15);
			cout<<hyper[index].integer();			
			k++; if(k>7){k=0;cout<<'\n';}

		}
		//checking for vectors
		else if(isupper(vector[0]))
		{
			VEC=hyper[index].vec();
			cout.width(15);
			cout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;cout<<'\n';}
		}
		else
		{
			//real variables
			cout.width(15);
			cout<<hyper[index].real();
			k++; if(k>7){k=0;cout<<'\n';}
		}
	}
	cout<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to 'tabout.asc'
//
//First label is time-since-launch-of-vehicle 'time', must be at round3[0]
// Accomodates real, integers (printed as real) and 3x1 vectors 
// Eight accross, unlimited down
// Data field 15 spaces, total width 120 spaces
//
//010114 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::tabout_data(ofstream &ftabout)
{

	int index;
	const char *integer;
	const char *vector;
	Matrix VEC(3,1);
	int k(0);
	int i(0);
	
	ftabout<<hyper5_name<<'\n';
	ftabout.setf(ios::left);

	//writing to 'tabout.asc' the variables from the 'Round3' class
	for(i=0;i<round3_scrn_count;i++)
	{
		index=round3_scrn_ind[i];
		//checking for integers
		integer=round3[index].get_type();
		vector=round3[index].get_name();
		if(!strcmp(integer,"int"))
		{
			ftabout.width(15);
			ftabout<<round3[index].integer();			
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=round3[index].vec();

			ftabout.width(15);
			ftabout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		else
		{
			//real variables
			ftabout.width(15);
			ftabout<<round3[index].real(); 
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
	}
	//writing to 'tabout.asc' the variables from the 'Hyper' class
	for(i=0;i<hyper_scrn_count;i++)
	{
		index=hyper_scrn_ind[i];
		//checking for integers
		integer=hyper[index].get_type();
		vector=hyper[index].get_name();
		if(!strcmp(integer,"int"))
		{
			ftabout.width(15);
			ftabout<<hyper[index].integer();			
			k++; if(k>7){k=0;ftabout<<'\n';}

		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=hyper[index].vec();
			ftabout.width(15);
			ftabout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		else
		{
			//real variables
			ftabout.width(15);
			ftabout<<hyper[index].real();
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
	}
	ftabout<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to 'ploti.asc', i=1,2,3...
//
//First label is time-since-launch-of-vehicle 'time', always at round3[0]
//Five accross, unlimited down
//Data field width 16 spaces, total width 80 spaces
//Labels longer than 8 characters will be truncated
//Accomodates 3x1 vectors
//
//010115 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::plot_banner(ofstream &fplot,const char *title)
{
	const char *buff1;
	char buff2[15];
	int label_length=13;
	int k(0);
	int m(0);
	int i(0);

	fplot<<"1"<<title<<" '"<<hyper5_name<<" ' "<< __DATE__ <<" "<< __TIME__ <<"\n";
	
	//determining the number vector variables
	for(i=0;i<nplot_hyper5;i++)
	{
		buff1=plot_hyper5[i].get_name();
		if(isupper(buff1[0])) m++;
	}
	//increase number of variables by vector components
	int nvariables=nplot_hyper5+2*m;
	
	fplot<<"  0  0 " <<nvariables<<"\n"; 

	//writing banner to plot file 'ploti.asc'
	for(i=0;i<nplot_hyper5;i++)
	{
		fplot.setf(ios::left);

		buff1=plot_hyper5[i].get_name();

		//truncating if more than 8 characters
		strncpy(buff2,buff1,label_length);
		buff2[13]=0;

		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int i=1;i<4;i++)
			{				
				fplot.width(strlen(buff2));
				fplot<<buff2;fplot.width(16-strlen(buff2));fplot<<i;
				k++;
				if(k>4){k=0;fplot<<'\n';}
			}
		}
		else
		{
			fplot.width(16);
			fplot<<buff2;
			k++;
			if(k>4){k=0;fplot<<'\n';}
		}
	}
	if((nvariables%5))fplot<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'round3[]' and hyper[] variables that are  
//output to 'ploti.asc'
//
//Output: Hyper::round3_plot_ind[], hyper_plot_ind[] 
//
//001213 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::plot_index_arrays()
{
	const char *test="plot";
	int k(0);
	int i(0);

	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),test))
		{
			round3_plot_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NHYPER;i++)
	{
		if(strstr(hyper[i].get_out(),test))
		{
			hyper_plot_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to 'ploti.asc', i=1,2,3...
//
//Accomodates real, integers (printed as real) and 3x1 vectors 
//five accross, unlimited down
//data field 16 spaces, total width 80 spaces
//
//010116 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::plot_data(ofstream &fplot,bool merge)
{

	int index;
	const char *integer;
	const char *vector;
	Matrix VEC(3,1);
	int k(0);
	int i(0);
	
	fplot.setf(ios::left);

	//writing to 'ploti.asc' the variables from the 'Round3' class
	for(i=0;i<round3_plot_count;i++)
	{
		index=round3_plot_ind[i];
		//checking for integers
		integer=round3[index].get_type();
		vector=round3[index].get_name();
		if(!strcmp(integer,"int"))
		{
			//casting integer to real variable
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<(double) round3[index].integer();			
			k++;
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=round3[index].vec();

			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(0,0);
			k++; 
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(1,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(2,0);
			k++;
		}
		else
		{
			//real variables
			if(merge&&(!index))
				//for merging files, time at last entry must be '-1'
			{
				fplot.width(16);
				fplot<<"-1.0";
				k++;
			}
			else
			{
				if(k>4){k=0;fplot<<'\n';}
				fplot.width(16);
				fplot<<round3[index].real(); 
				k++;
			}
		}
	}
	//writing to 'ploti.asc' the variables from the 'Hyper' class
	for(i=0;i<hyper_plot_count;i++)
	{
		index=hyper_plot_ind[i];
		//checking for integers
		integer=hyper[index].get_type();
		vector=hyper[index].get_name();
		if(!strcmp(integer,"int"))
		{
			//casting integer to real variable
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<(double) hyper[index].integer();			
			k++;
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=hyper[index].vec();
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(0,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(1,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(2,0);
			k++;
		}
		else
		{
			//real variables
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<hyper[index].real();
			k++;
		}
	}
	fplot<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Watching for and executing events
// 
//Max number of events set by global constant NEVENT
//Max number of variables in any event set by global constant NVAR
//Each event has to be surrounded in 'input.asc' by 'IF' and 'ENDIF'
//Event criteria and variables can be 'double' or 'int'
//New variable-values are subsituted before calling modules
//
//010125 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::event(const char *options)
{
	Variable *watch_variable_ptr;
	double current_value=0;
	int current_value_int=0;
	double crit_value=0;
	int crit_value_int=0;
	char oper;
	const char *intest=NULL;

	//initializing event flag ('Hyper' member)
	event_epoch=false;

	//returning if no events occur for this vehicle object
	//or if all events have already occurred
	if(!event_total)return;
	
	//getting watch variable's current value and critical value
	watch_variable_ptr=event_ptr_list[nevent]->get_variable();
	crit_value=event_ptr_list[nevent]->get_value();

	//testing for integer
	intest=watch_variable_ptr->get_type();
	if(!strcmp(intest,"int"))
	{
		current_value_int=watch_variable_ptr->integer();
		crit_value_int=(int)crit_value;
	}
	else
		current_value=watch_variable_ptr->real();
	//getting relational operator
	oper=event_ptr_list[nevent]->get_operator();

	//checking if event occurred
	if(oper=='<')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int<crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value<crit_value)
				event_epoch=true;
		}
	}
	else if(oper=='=')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int==crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value==crit_value)event_epoch=true;
		}
	}
	else if(oper=='>')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int>crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value>crit_value)
				event_epoch=true;
		}
	}

	//loading new variables
	if(event_epoch)
	{
		int *round3_index_list;
		double *round3_value_list;
		int round3_size(0);
		int *hyper_index_list;
		double *hyper_value_list;
		int hyper_size(0);
		int index(0);
		const char *integer;
		double value(0);
		int value_int(0);
		int i(0);

		round3_index_list=event_ptr_list[nevent]->get_round3_indices();
		round3_value_list=event_ptr_list[nevent]->get_round3_values();
		round3_size=event_ptr_list[nevent]->get_round3_size();

		hyper_index_list=event_ptr_list[nevent]->get_hyper_indices();
		hyper_value_list=event_ptr_list[nevent]->get_hyper_values();
		hyper_size=event_ptr_list[nevent]->get_hyper_size();

		for(i=0;i<round3_size;i++)
		{
			index=round3_index_list[i];
			value=round3_value_list[i];

			integer=round3[index].get_type();
			if(!strcmp(integer,"int"))
			{
				value_int=(int)value;
				round3[index].gets(value_int);
			}
				round3[index].gets(value); // new value assigned
		}
		for(i=0;i<hyper_size;i++)
		{
			index=hyper_index_list[i];
			value=hyper_value_list[i];

			integer=hyper[index].get_type();
			if(!strcmp(integer,"int"))
			{
				value_int=(int)value;
				hyper[index].gets(value_int);
			}
				hyper[index].gets(value); // new value assigned
		}
				
		//writing event message to console
		double time=round3[0].real();
		const char *name=watch_variable_ptr->get_name();
		if(strstr(options,"y_events"))
		{
			cout<<" *** Event #"<<nevent+1<<'\t'<<hyper5_name<<'\t'<<"time = "<<time
				<<"\tsec;  criteria:  "<<name<<" "<<oper<<" "<<crit_value<<"\t***\n";
		}
		//increment event number
		nevent++;
		//reset 'event_total' to zero after last event has occured
		if(nevent==event_total)event_total=0;	
	}
}
///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'hyper' and 'round3' module-variable arrays
//
//010126 Created by Peter Zipfel
//020911 Added module-variable error flagging, PZi
//030627 Adapted to CRUISE simulation, PZi
//060929 Modification to accomodate C++8, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::document(ostream &fdoc,const char *title,Document *doc_hyper5)
{
	int i(0);
	int j(0);

	fdoc<<"*********************************************************************************************************************\n";
	fdoc<<"********************************************** HYPER5 **************************************************************\n";
	fdoc<<"*********************************************************************************************************************\n";
	fdoc<<"\n"<<"*** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***\n\n";
//	fdoc<<"01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456\n";
	fdoc<<"\n\n                                       Hyper Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                    DEFINITION                       |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NHYPER;i++)
	{
		for(j=0;j<i;j++){
			if(!strcmp(hyper[i].get_name(),hyper[j].get_name())&&strcmp(hyper[i].get_name(),"empty"))
				hyper[i].put_error(name_error_code);
		}				
		if(!strcmp(hyper[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in hyper[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(hyper[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in hyper[] array, see 'doc.asc' ***\n"; 

				fdoc<<hyper[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(hyper[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<hyper[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<hyper[i].get_name();
		}
		fdoc.width(54);fdoc<<hyper[i].get_def();
		fdoc.width(13);fdoc<<hyper[i].get_mod();
		fdoc.width(10);fdoc<<hyper[i].get_role();
		fdoc<<hyper[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}

	fdoc<<"\n\n                                       Round3 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	  
	for(i=0;i<NROUND3;i++)
	{
		for(j=0;j<i;j++){
			if(!strcmp(round3[i].get_name(),round3[j].get_name())&&strcmp(round3[i].get_name(),"empty"))
				round3[i].put_error(name_error_code);
		}				
		if(!strcmp(round3[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in round3[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(round3[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in round3[] array, see 'doc.asc' ***\n"; 
		
		fdoc<<round3[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(round3[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<round3[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<round3[i].get_name();
		}
		fdoc.width(54);fdoc<<round3[i].get_def();
		fdoc.width(13);fdoc<<round3[i].get_mod();
		fdoc.width(10);fdoc<<round3[i].get_role();
		fdoc<<round3[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}

	//building doc_hyper5[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NHYPER;i++){
		if(strcmp(hyper[i].get_name(),"empty")){
			doc_hyper5[counter].put_doc_offset(counter);
			doc_hyper5[counter].put_name(hyper[i].get_name()); 

//z060929		doc_hyper5[counter].put_type(hyper[i].get_type());

			//Previous line breakes in MSC++8 debugger because 'put_type()' function
			// was updated to 'strcpy()' from 'strcpy()
			// Solution: when "int" is not present (because not an integer module-variable),
			//   'dum' keeps the original I-accent characters, which are equivalent to '-51'; 
			//   therefore, do not execute 'put_type()' if module-variable is not an integer
			const char *dum;
			dum=hyper[i].get_type();
			if(*dum!=-51)
				doc_hyper5[counter].put_type(dum);
//z060929-end

			doc_hyper5[counter].put_def(hyper[i].get_def());
			doc_hyper5[counter].put_mod(hyper[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NROUND3;i++){
		if(strcmp(round3[i].get_name(),"empty")){
			doc_hyper5[counter].put_doc_offset(counter);
			doc_hyper5[counter].put_name(round3[i].get_name());

//z060929		doc_hyper5[counter].put_type(round3[i].get_type());

			const char *dum;
			dum=round3[i].get_type();
			if(*dum!=-51)
				doc_hyper5[counter].put_type(dum);
//z060929-end

			doc_hyper5[counter].put_def(round3[i].get_def());
			doc_hyper5[counter].put_mod(round3[i].get_mod());
			counter++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'round3[]' and hyper[] variables 
// that are output to 'combus' 'data'  
//
//Output: Hyper::round3_com_ind[], hyper_com_ind[] 
//
//010210 Created by Peter Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::com_index_arrays()
{
	const char *test="com";
	int k=0;
	int i(0);

	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),test))
		{
			round3_com_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NHYPER;i++)
	{
		if(strstr(hyper[i].get_out(),test))
		{
			hyper_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' with 'HYPER5' data
//
//Uses C-code 'sprintf' function to convert 'int' to 'char'
//Differs from 'loading_packet' only by initializing 'status=1'
//
//Output by 'return packet'
//
//010401 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Packet Hyper::loading_packet_init(int num_hyper,int num_target,int num_satellite)
{
	string id;
	char object[4];
	static int h_count(0);
	int index(0);
	int i(0);
	
	h_count++;
	if(h_count==(num_hyper+1))h_count=1;
	sprintf(object,"%i",h_count);
	id="h"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_hyper5[i]=round3[index]; 
	}
	for(int j=0;j<hyper_com_count;j++)
	{
		index=hyper_com_ind[j];
		com_hyper5[i+j]=hyper[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_hyper5);
	packet.set_ndata(ncom_hyper5);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' with 'HYPER5' data
//
//Uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010206 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Packet Hyper::loading_packet(int num_hyper,int num_target,int num_satellite)
{
	int index(0);
	int i(0);
	int j(0);

/*	string id;
	char object[4];
	static int h_count=0;
	
	h_count++;
	if(h_count==(num_hyper+1))h_count=1;
	sprintf(object,"%i",h_count);
	id="h"+string(object);
*/
	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_hyper5[i]=round3[index];
	}
	for(j=0;j<hyper_com_count;j++)
	{
		index=hyper_com_ind[j];
		com_hyper5[i+j]=hyper[index];
	}
	//refreshing the packet
//	packet.set_id(id);
	packet.set_data(com_hyper5);
	packet.set_ndata(ncom_hyper5);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Reading tables from table decks
//
//Supports 1, 2, 3 dim tables stored seperately in data decks
//Keying on AERO_DECK and PROP_DECK in 'input.asc' this function  reads the tables
// from the data files and stores them in 'Hyper::Datadeck aerotable' and 
// 'Hyper::Datadeck proptable'. In the modules, table look up is carried out by 
// double value=aerodeck.look_up(string name, double var1);  
// double value=aerodeck.look_up(string name, double var1,double var2);  
// double value=aerodeck.look_up(string name, double var1,double var2,double var3);  
//
//To add new tables, just include them in the files of  AERO_DECK and PROP_DECK
//For debugging puposes un-comment the print out provision of the tables below
//
//030721 Created by Peter H Zipfel
//060426 Corrected line 1401 (replaced 'if else' by 'if'), PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::read_tables(const char *file_name,Datadeck &datatable)
{

	char line_clear[CHARL];
	char temp[CHARN];	//buffer for table data
	string table_deck_title;
	int table_count(0);
	int table_dim(0);
	double value(0);
	int file_ptr=NULL;
	int var_dim[3]={1,1,1,};
	int tt(0);

	//opening data-deck file stream
	ifstream tbl_stream(file_name);

	if(tbl_stream.fail())
		{cerr<<"*** Error: File stream '"<<file_name<<"' failed to open (check spelling) ***\n";exit(1);}

	//determing the total # of tbl_stream
	while(!tbl_stream.eof())
	{
		tbl_stream>>temp;
		if(!strcmp(temp,"TITLE")){
			tbl_stream.getline(line_clear,CHARL,'\n');
			table_deck_title=line_clear;
		}
		if(strstr(temp,"DIM"))
			table_count++;

	} //EOF reached of data-deck

	//removing EOF bit (-1)
	tbl_stream.clear();
	//rewinding to beginning
	tbl_stream.seekg(ios::beg);

	//creating pointer array table 
	datatable.set_title(table_deck_title);
	datatable.set_capacity(table_count);
	datatable.alloc_mem();

	//discarding all entries until first DIM
	do{
		tbl_stream>>temp;
	}while(!strstr(temp,"DIM"));

	//loading tables one at a time
	for(int t=0;t<table_count;t++){

		//creating and allocating memory to object 'table'
		try{table=new Table;}
		catch(bad_alloc xa){cerr<< "*** Allocation failure of 'table' ***\n";exit(1);}
			
		//extracting table dimension
		//at this point 'temp' is holding xDIM
		int dim_check(0);
		char dim_buff[2];
		int table_dim(0);
		strncpy(dim_buff,temp,1);
		//converting character to integer
		dim_check=sscanf(dim_buff,"%d",&table_dim);
		table->set_dim(table_dim);
								
		//extracting table name
		tbl_stream>>temp;
		table->set_name(temp);
		tbl_stream.getline(line_clear,CHARL,'\n');

		//extracting dimensions of independent variables
		var_dim[0]=1;var_dim[1]=1;var_dim[2]=1;
		for(tt=0;tt<table_dim;tt++){
			tbl_stream>>temp;
			tbl_stream>>var_dim[tt];
			if(tt==0)
				table->set_var1_dim(var_dim[tt]);
			if(tt==1)
				table->set_var2_dim(var_dim[tt]);
			if(tt==2)
				table->set_var3_dim(var_dim[tt]);
		}
		tbl_stream.getline(line_clear,CHARL,'\n');

		//allocating memory for variables and data arrays
		table->var1_values=new double [var_dim[0]];
		table->var2_values=new double [var_dim[1]];
		table->var3_values=new double [var_dim[2]];
		table->data=new double[var_dim[0]*var_dim[1]*var_dim[2]];

		//determining max number of rows of data
		int num_rows=var_dim[0];
		if(var_dim[0]<var_dim[1]) num_rows=var_dim[1];
		if(var_dim[2]>num_rows) num_rows=var_dim[2];

		//reading num_row of data 
		for(tt=0;tt<num_rows;tt++){

			//loading 1.variable values
			if(tt<var_dim[0]){
				tbl_stream>>value;
				table->set_var1_value(tt,value);
			}

			//loading 2.variable values, but bypass if default dimension one
			if(tt<var_dim[1]&&var_dim[1]!=1){
				tbl_stream>>value;
				table->set_var2_value(tt,value);
			}

			//loading 3.variable values, but bypass if default dimension one
			if(tt<var_dim[2]&&var_dim[2]!=1){
				tbl_stream>>value;
				table->set_var3_value(tt,value);
			}

			//loading tabular data, which in all cases has only 'var_dim[0]' rows
			if(tt<var_dim[0]){

				//read one row of data
				for(int ttt=0;ttt<var_dim[1]*var_dim[2];ttt++){
					tbl_stream>>value;
					table->set_data(tt*var_dim[1]*var_dim[2]+ttt,value);
				}
			}
		}//end of reading data

		//loading table into 'Datadeck' pointer array 'Table **table_ptr'
		datatable.set_counter(t);
		datatable.add_table(*table);
		tbl_stream>>temp; //reading next DIM entry
		
	}//end of 'for' loop, finished loading all tables

	/*/////////////////////// DIAGNOSTICS //////////////////////////////////////
	//Diagnostic display of tables on console
	int num_tables=datatable.get_capacity();
	string deck_title=datatable.get_title();
	cout<<"\n ********************* Data Deck *********************\n";
	cout<<"TITLE "<<deck_title<<"\n\n";

	for(int ti=0;ti<num_tables;ti++){
		int tbl_dim=datatable[ti]->get_dim();

		if(tbl_dim==1){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			cout<<"NX1 "<<var1_dim<<'\n';
			for(int tii=0;tii<var1_dim;tii++){
				cout<<datatable[ti]->var1_values[tii]<<"\t"<<datatable[ti]->data[tii]<<endl;
			}
		cout<<'\n';
		}//end of 1-dim tables

		if(tbl_dim==2){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			int var2_dim=datatable[ti]->get_var2_dim();
			cout<<"NX1 "<<var1_dim<<"   NX2 "<<var2_dim<<'\n';

			//find maximum number of rows
			int num_rows=var1_dim;
			if(num_rows<var2_dim)
				num_rows=var2_dim;

			for(int tii=0;tii<num_rows;tii++){
				if(tii<var1_dim)
					cout<<datatable[ti]->var1_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var2_dim) 
					cout<<datatable[ti]->var2_values[tii]<<"\t";
				else 
					cout<<"*\t";

				if(tii<var1_dim){
					for(int tk=0;tk<var2_dim;tk++)
						cout<<datatable[ti]->data[tk+tii*var2_dim]<<'\t';
					cout<<endl;
				}
				else{
					for(int tk=0;tk<var2_dim;tk++)
						cout<<"*\t";
					cout<<endl;
				}

			}
		cout<<'\n';
		}//end of 2-dim tables

		if(tbl_dim==3){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			int var2_dim=datatable[ti]->get_var2_dim();
			int var3_dim=datatable[ti]->get_var3_dim();
			cout<<"NX1 "<<var1_dim<<"   NX2 "<<var2_dim<<"   NX3 "<<var3_dim<<'\n';

			//find maximum number of rows
			int num_rows=var1_dim;
			if(num_rows<var2_dim)
				num_rows=var2_dim;
			if(num_rows<var3_dim)
				num_rows=var3_dim;

			for(int tii=0;tii<num_rows;tii++){
				if(tii<var1_dim)
					cout<<datatable[ti]->var1_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var2_dim) 
					cout<<datatable[ti]->var2_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var3_dim) 
					cout<<datatable[ti]->var3_values[tii]<<"\t";
				else 
					cout<<"*\t";

				if(tii<var1_dim){
					for(int tk=0;tk<var2_dim*var3_dim;tk++)
						cout<<datatable[ti]->data[tk+tii*var2_dim*var3_dim]<<'\t';
					cout<<endl;
				}
				else{
					for(int tk=0;tk<var2_dim*var3_dim;tk++)
						cout<<"*\t";
					cout<<endl;
				}
			}
		cout<<'\n';
		}//end of 3-dim tables
	}//end of diagnostic table print-out
	/*//////////////////////////////////////////////////////////////////////////

}