#pragma once

#ifndef _XML_IMPORTER_H_
#define _XML_IMPORTER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <list>
#include <vector>

class XML_reader
{
public:
// constructor
	XML_reader(std::string name_file);
	XML_reader(std::ifstream* file_stream, std::string terminator) { this->reading_loop(file_stream, terminator); }
// methods
	static std::list<std::string> splitta_riga(std::string& riga);
	static std::vector<std::vector<float>> Load(const std::string& file);
	std::vector<std::vector<std::string>>	Get_raw(std::string name_element);

	int										Get_number_of_elements(std::string name_element);
// iterator methods
	class Element_iterator;
	Element_iterator Get_iterator(std::string field_name); //iterator to scroll every field with a certain name
private:
	struct Element
	{
		std::string name;
		std::vector<std::vector<std::string>> content;
	};
// methods
	void reading_loop(std::ifstream* file_stream, std::string terminator);
	Element* Find_element(std::string& name_element);
// data
	std::string				mName_object;
	std::list<Element>		mElements;
};

class XML_reader::Element_iterator {
public:
	Element_iterator(XML_reader* pread, std::string& field_name);
	bool Found_at_least_one() { return this->found_at_least_one; };
	void Increment();
	std::vector<std::vector<std::string>>* Get_pointer_to_data();
private:
	std::list<Element>::iterator  it_El;
	XML_reader*  pReader;
	std::string  name_field;
	bool found_at_least_one;
};

#endif