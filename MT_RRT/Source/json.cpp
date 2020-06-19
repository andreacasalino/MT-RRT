#include "../Header/json.h"
#include <list>
#include <fstream>
using namespace std;

namespace MT_RTT
{

	/////////////////////////////////////////////////
	//				--- PARSE ---                  //
	/////////////////////////////////////////////////

	string extract(const string& s, const size_t& po, const size_t& pf) { return string(s, po, pf - po + 1); };

	string get_compact_JSON(const string& JSON_raw) {
		string JSON_compact;
		JSON_compact.reserve(JSON_raw.size());
		size_t K = JSON_raw.size();
		for (size_t k = 0; k < K; ++k) {
			if ((JSON_raw[k] != '\n') && (JSON_raw[k] != ' '))
				JSON_compact.push_back(JSON_raw[k]);
		}
		return JSON_compact;
	};

	list<size_t> find(const string& JSON, const char& c) {
		list<size_t> pos;
		size_t p = 0;
		while (true) {
			p = JSON.find(c, p);
			if (p == string::npos) break;
			pos.push_back(p);
			++p;
		}
		return pos;
	};

	vector<float> parse_array(const string& to_parse) {
		vector<float> vals;
		if (to_parse.compare("[]") != 0) {
			size_t P = to_parse.size() - 1;
			string temp;
			for (size_t p=1; p < P; ++p) {
				if (to_parse[p] == ',') {
					vals.push_back((float)atof(temp.c_str()));
					temp.clear();
				}
				else temp.push_back(to_parse[p]);
			}
			vals.push_back((float)atof(temp.c_str()));
		}
		return vals;
	};

	vector<vector<float>> parse_arrays(const string& to_parse) {
		vector<vector<float>> vals;
		if (to_parse.compare("null") == 0) return vals;
		if (to_parse.compare("NULL") == 0) return vals;
		if (to_parse.compare("Null") == 0) return vals;
		if (to_parse.compare("[[]]") == 0) return vals;
		if (to_parse.compare("[]") == 0) return vals;
		size_t P = to_parse.size() - 1;
		string temp;
		for (size_t p=1; p < P; ++p) {
			if ((to_parse[p] == ',') && (to_parse[p - 1] == ']')) {
				vals.emplace_back();
				vals.back() = parse_array(temp);
				temp.clear();
			}
			else temp.push_back(to_parse[p]);
		}
		vals.emplace_back();
		vals.back() = parse_array(temp);
		return vals;
	};

	vector<vector<float>> parse_field(const string& JSON) {
		if (JSON[1] == '[') return parse_arrays(JSON);
		else				return { parse_array(JSON) };
	}

	vector<json_parser::field> json_parser::parse_JSON(const string& JSON) {
		vector<field> temp;
		string JSON_comp = get_compact_JSON(JSON);
		auto apici = find(JSON_comp, '\"');
		temp.reserve(apici.size() / 2);
		while (!apici.empty()) {
			temp.emplace_back();

			size_t p = apici.front(); apici.pop_front();
			size_t pf = apici.front(); apici.pop_front();
			temp.back().name.reserve(pf - p);
			++p;
			for (p=p; p < pf; ++p) temp.back().name.push_back(JSON_comp[p]);

			if (apici.size() == 0) {
				string ext = extract(JSON_comp, pf + 2, JSON_comp.size() - 2);
				temp.back().values = parse_field(ext);
			}
			else {
				string ext = extract(JSON_comp, pf + 2, apici.front() - 2);
				temp.back().values = parse_field(ext);
			}
		}
		return temp;
	};



	const std::vector<std::vector<float>>* json_parser::get_field(const vector<field>& fields, const std::string& name_to_search) {

		for (size_t k = 0; k < fields.size(); ++k) {
			if (fields[k].name.compare(name_to_search) == 0) return &fields[k].values;
		}
		return NULL;

	}

	std::string json_parser::get_JSON_from_file(const std::string& file_location) {

		string temp, temp2;
		ifstream f(file_location);
		if (!f.is_open()) throw 0;
		while (!f.eof()) {
			getline(f, temp2);
			temp += temp2;
		}
		f.close();
		return temp;

	}

	/////////////////////////////////////////////////
	//				--- Load  ---                  //
	/////////////////////////////////////////////////

	std::string	json_parser::load_JSON(const float* buffer, const size_t& Size) {
		string temp = "[";
		if (Size > 0) {
			temp += to_string(buffer[0]);
			for (size_t k = 1; k < Size; ++k)
				temp += "," + to_string(buffer[k]);
		}
		temp += "]";
		return temp;
	}

	std::string	json_parser::load_JSON(const std::vector<std::vector<float>>& values) {
		string temp = "[";
		if (!values.empty()) {
			auto temp2 = json_parser::load_JSON(&values[0][0], values[0].size());
			temp += temp2;
			for (size_t k = 1; k < values.size(); ++k) {
				auto temp3 = json_parser::load_JSON(&values[k][0], values[k].size());
				temp += "," + temp3;
			}
		}
		temp += "]";
		return temp;
	}

	std::string	json_parser::load_JSON(const float* buffer, const std::vector<size_t>& Sizes) {
		string temp = "[";
		size_t s = 0;
		temp += json_parser::load_JSON(&buffer[s], Sizes.front());
		s += Sizes[0];
		for (size_t k = 1; k < Sizes.size(); ++k) {
			auto temp2 = json_parser::load_JSON(&buffer[s], Sizes[k]);
			temp += "," + temp2;
			s += Sizes[k];
		}
		temp += "]";
		return temp;
	}

	string stringify_field(const json_parser::field& f) {
		string temp = "\"" + f.name + "\":";
		temp += json_parser::load_JSON(f.values);
		return temp;
	}

	string json_parser::load_JSON(const std::vector<field>& fields) {
		string temp = "{";
		if (!fields.empty()) {
			auto temp2 = stringify_field(fields[0]);
			temp += temp2;
			for (size_t k = 1; k < fields.size(); ++k) {
				auto temp3 = stringify_field(fields[k]);
				temp += "," + temp3;
			}
		}
		temp += "}";
		return temp;
	}

}