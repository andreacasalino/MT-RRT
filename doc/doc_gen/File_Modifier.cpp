#include "File_Modifier.h"
#include <fstream>
using namespace std;

File_Modifier::File_Modifier(const std::string& source) {
	ifstream f(source);
	if (!f.is_open()) {
		system("source file not found");
		throw 0;
	}
	while (!f.eof()) {
		this->contents.emplace_back();
		getline(f, this->contents.back());
	}
	f.close();
}

void File_Modifier::reprint(const std::string& destination) {
	ofstream f(destination);
	if (!f.is_open()) {
		system("destination file not found");
		throw 0;
	}
	auto it_end = this->contents.end();
	auto it = this->contents.begin();
	f << *it;
	it++;
	for (it; it != it_end; it++) f << endl << *it;
	f.close();
}

std::string File_Modifier::zip(const std::list<std::string>& to_zip) {
	string temp;
	auto it_end = to_zip.end();
	auto it = to_zip.begin();
	temp = *it;
	it++;
	for (it; it != it_end; it++) temp += "\n" +  *it;
	return move(temp);
}

std::list<std::string>::iterator File_Modifier::__find_line(const std::string& to_search){
	auto it_end = this->contents.end();
	for (auto it = this->contents.begin(); it != it_end; it++) {
		if (it->compare(to_search) == 0) return it;
	}
	system("echo line not found");
	throw 0;
}

void File_Modifier::add_after(const std::string& line_involved, const std::string& to_add) {
	auto line = this->__find_line(line_involved);
	line++;
	this->contents.insert( line, to_add);
}

void File_Modifier::add_before(const std::string& line_involved, const std::string& to_add) {
	auto line = this->__find_line(line_involved);
	this->contents.insert(line, to_add);
}

void File_Modifier::replace(const std::string& line_to_replace, const std::string& to_write) {
	auto line = this->__find_line(line_to_replace);
	*line = to_write;
}