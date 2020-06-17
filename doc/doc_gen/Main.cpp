#include "File_Modifier.h"
#include <iostream>
#include <fstream>
using namespace std;

void bat_command(const list<string>& comms);

int main() {

	system("RD \".\\latex\" /S /Q");
	system("\"C:\\Program Files\\doxygen\\bin\\doxygen\" doxy_config");

	{
		File_Modifier FM_refman("./latex/refman.tex");
		File_Modifier FM_packages("packages.tex");
		FM_refman.add_after("\\renewcommand{\\numberline}[1]{#1~}", File_Modifier::zip(FM_packages.get_contents()));
		File_Modifier FM_add_sections("additional_Sections.tex");
		FM_refman.add_after("%--- Begin generated contents ---", File_Modifier::zip(FM_add_sections.get_contents()));
		FM_refman.add_before("\\end{document}", File_Modifier::zip({ "\\bibliography{../ref}{}", "\\bibliographystyle{plain}" }));
		FM_refman.reprint("./latex/refman.tex");
	}

	{
		File_Modifier FM_make("./latex/make.bat");
		FM_make.add_before("setlocal enabledelayedexpansion", "bibtex refman");
		FM_make.replace("set count=8", "set count=4");
		FM_make.add_before("cd /D %Dir_Old%", "COPY refman.pdf \"../documentation.pdf\"");
		FM_make.replace("cd /D %Dir_Old%", "");
		FM_make.replace("set Dir_Old=", "");
		FM_make.reprint("./latex/make.bat");
	}

	bat_command({"cd latex", "make.bat"});

	system("COPY \"./latex/refman.pdf\" \"documentation.pdf\"");
	system("RD latex /S /Q");

	return 0;
}

void bat_command(const list<string>& comms) {
	ofstream f("__temp.bat");
	for (auto it = comms.begin(); it != comms.end(); it++)  f << *it << endl;
	f.close();
	system("__temp.bat");
	system("DEL __temp.bat");
}