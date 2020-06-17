/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#include <string>
#include <fstream>
#include <iostream>

void replacer(std::string& involved, const char& to_find, const char& to_set) {
	size_t K = involved.size();
	char* buff = &involved[0];
	for (size_t k = 0; k < K; k++) {
		if (buff[k] == to_find) buff[k] = to_set;
	}
};

void Log_creator(const std::string& template_location, const std::string& destination, std::string& problem_json, std::string& result_json) {

	std::ifstream fi(template_location);
	if (!fi.is_open()) {
		std::cout << "template file not found\n";
		throw 0;
	}
	std::ofstream fo(destination);
	if (!fo.is_open()) {
		std::cout << "echo impossible to create destination file\n";
		throw 1;
	}

	replacer(problem_json, '\n', ' ');
	replacer(result_json, '\n', ' ');

	std::string temp;
	while (!fi.eof()) {
		getline(fi, temp);
		if (temp.compare("let problem=") == 0)		fo << "let problem='" << problem_json << "';" << std::endl;
		else if(temp.compare("let result=") == 0)	fo << "let result='" << result_json << "';" << std::endl;
		else										fo << temp << std::endl;
	}
	fi.close();
	fo.close();

}

std::string get_content_of_file(const std::string& file) {

	std::string tot, temp;
	std::ifstream f(file);
	while (!f.eof()) {
		getline(f, temp);
		tot += temp + "\n";
	}
	f.close();
	return std::move(tot);

}
