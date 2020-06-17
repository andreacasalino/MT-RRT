#ifndef __FILE_MODIFIER_H__
#define __FILE_MODIFIER_H__

#include <list>
#include <string>

class File_Modifier {
public:
	File_Modifier(const std::string& source);
	const std::list<std::string>& get_contents() { return this->contents; };
	void reprint(const std::string& destination);

	static std::string zip(const std::list<std::string>& to_zip);

	void add_after(const std::string& line_involved, const std::string& to_add);
	void add_before(const std::string& line_involved, const std::string& to_add);
	void replace(const std::string& line_to_replace, const std::string& to_write);
private:
	std::list<std::string>::iterator __find_line(const std::string& to_search);

	std::list<std::string> contents;
};

#endif