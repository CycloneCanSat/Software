using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Application
{
	class Program
	{
		public static void RemoveArrayItem<T>(ref T[] inArray, int inIndex)
		{
			T[] newArray = new T[inArray.Length - 1];
			for (int i = 0, j = 0; i < newArray.Length; i++, j++)
			{
				if (i == inIndex)
				{
					j++;
				}
				newArray[i] = inArray[j];
			}
			inArray = newArray;
		}
		struct people
		{
			public int mark;
			public string name;
		}
		static people[] people2 = new people[0];
		static int numberOfPeople = 0;
		static void EditEntries()
		{
			DisplayEntries ();
			Console.WriteLine ("Enter which entry you wish to change");
			int input = Convert.ToInt32 (Console.ReadLine ());
			input--;
			Console.WriteLine ("Enter the new entry in the form: name mark");
			string inp = Console.ReadLine ();
			string[] seperatedInput = inp.Split (' ');
			string nameToBeReplaced = seperatedInput [0];
			int markToBeReplaced = Convert.ToInt32 (seperatedInput [1]);
			people2 [input].name = nameToBeReplaced;
			people2 [input].mark = markToBeReplaced;
			Console.Clear ();
			DisplayEntries ();
		}
		static void EnterNewEntries()
		{
			Console.WriteLine("Enter entries in the form: Name Mark - to exit enter x");
			string input = Console.ReadLine();
			while (input != "x")
			{
				string[] inputArr = input.Split(' ');
				string nameTemp = inputArr[0];
				int markTemp = 0;
				if(!int.TryParse(inputArr[1], out markTemp))
				{
					Console.WriteLine ("[Mark input was invalid - mark will be set as 0]");
				}
				int i = numberOfPeople + 1;
				Array.Resize(ref people2, i);
				people2[numberOfPeople].name = nameTemp;
				people2[numberOfPeople].mark = markTemp;
				numberOfPeople++;
				input = Console.ReadLine();
			}
			Console.Clear ();
		}
		static void DeleteEntries()
		{
			DisplayEntries();
			Console.WriteLine("Enter which entry to delete");
			int input = Convert.ToInt32(Console.ReadLine());
			RemoveArrayItem(ref people2, (input - 1));
			Console.Clear ();
			DisplayEntries ();
		}
		static void DisplayEntries()
		{
			for (int i = 0; i < people2.Length; i++)
			{
				int j=i+1;
				Console.WriteLine(j + "\t" + people2[i].name + "\t" + people2[i].mark);
			}
		}
		static void SortByName()
		{
			Array.Sort<people> (people2, (x, y) => x.name.CompareTo (y.name));
			Console.Clear ();
			DisplayEntries ();
		}
		static void SortByMark()
		{
			Array.Sort<people> (people2, (x, y) => x.mark.CompareTo (y.mark));
			Console.Clear ();
			DisplayEntries ();
		}
		static void Main(string[] args)
		{
			Console.Write("Menu:");
			Console.WriteLine("\t1 - Enter new entries");
			Console.WriteLine ("\t2 - Edit entry");
			Console.WriteLine("\t3 - Delete entries");
			Console.WriteLine("\t4 - Display entries");
			Console.WriteLine("\t5 - Sort entries by name");
			Console.WriteLine("\t6 - Sort entries by mark");
			Console.WriteLine("\t7 - Exit\n");
			int input = Convert.ToInt32(Console.ReadLine());
			while(input != 7)
			{
				switch(input)
				{
				case 1:
					EnterNewEntries();
					break;
				case 2:
					EditEntries ();
					break;
				case 3:
					DeleteEntries();
					break;
				case 4:
					DisplayEntries();
					break;
				case 5:
					SortByName();
					break;
				case 6:
					SortByMark();
					break;
				default:
					Console.WriteLine("Invalid input. Please try again.");
					break;
				}
				Console.Write("\nMenu:");
				Console.WriteLine("\t1 - Enter new entries");
				Console.WriteLine ("\t2 - Edit entry");
				Console.WriteLine("\t3 - Delete entries");
				Console.WriteLine("\t4 - Display entries");
				Console.WriteLine("\t5 - Sort entries by name");
				Console.WriteLine("\t6 - Sort entries by mark");
				Console.WriteLine("\t7 - Exit\n");
				input = Convert.ToInt32(Console.ReadLine());
			}
		}
	}
}
