#include <socketcan_interface/string.h>
#include <iomanip>

namespace can {

bool hex2dec(uint8_t& d, const char& h) {
	if ('0' <= h && h <= '9')
		d = h - '0';
	else if ('a' <= h && h <= 'f')
		d = h - 'a' + 10;
	else if ('A' <= h && h <= 'F')
		d = h - 'A' + 10;
	else
		return false;

	return true;
}

bool hex2buffer(std::string& out, const std::string& in_raw, bool pad) {
	std::string in(in_raw);
	if ((in.size() % 2) != 0) {
		if (pad)
			in.insert(0, "0");
		else
			return false;
	}
	out.resize(in.size() >> 1);
	for (size_t i = 0; i < out.size(); ++i) {
		uint8_t hi, lo;
		if (!hex2dec(hi, in[i << 1]) || !hex2dec(lo, in[(i << 1) + 1]))
			return false;

		out[i] = (hi << 4) | lo;
	}
	return true;
}

bool dec2hex(char& h, const uint8_t& d, bool lc) {
	if (d < 10) {
		h = '0' + d;
	} else if (d < 16 && lc) {
		h = 'a' + (d - 10);
	} else if (d < 16 && !lc) {
		h = 'A' + (d - 10);
	} else {
		h='?';
		return false;
	}
	return true;
}

std::string byte2hex(const uint8_t& d, bool pad, bool lc) {
	uint8_t hi = d >> 4;
	char c=0;
	std::string s;
	if (hi || pad) {
		dec2hex(c, hi, lc);
		s += c;
	}
	dec2hex(c, d & 0xf, lc);
	s += c;
	return s;
}

std::string buffer2hex(const std::string& in, bool lc) {
	std::string s;
	s.reserve(in.size() * 2);
	for (size_t i = 0; i < in.size(); ++i) {
		std::string b = byte2hex(in[i], true, lc);
		if (b.empty())
			return b;

		s += b;
	}
	return s;
}

std::string tostring(const Header& h, bool lc) {
	std::string s;
	s.reserve(8);
	std::stringstream buf;
	buf << std::hex;
	if (lc)
		buf << std::nouppercase;
	else
		buf << std::uppercase;

	if (h.is_extended)
		buf << std::setfill('0') << std::setw(8);

	buf << (h.fullid() & ~Header::EXTENDED_MASK);
	return buf.str();
}

uint32_t tohex(const std::string& s) {
		unsigned int h = 0;
		std::stringstream stream;
		stream << std::hex << s;
		stream >> h;
		return h;
}

Header toheader(const std::string& s) {
	unsigned int h = tohex(s);
	unsigned int id = h & Header::ID_MASK;
	return Header(id, h & Header::EXTENDED_MASK || (s.size() == 8 && id >= (1<<11)),
			h & Header::RTR_MASK, h & Header::ERROR_MASK);
}

std::string tostring(const Frame& f, bool lc) {
	std::string s;
	std::string sep = "#";
	s.resize(f.dlc);
	for (uint8_t i = 0; i < f.dlc; ++i) {
		s[i] = f.data[i];
	}

	if (f.is_fd) {
		char flags;
		dec2hex(flags, f.flags, false);
		sep += "#";
		sep += flags;
	}

	return tostring((const Header&) (f), lc) + sep + buffer2hex(s, lc);
}

Frame toframe(const std::string& s) {
	size_t sep = s.find('#');
	bool is_fd = false;
	uint8_t fd_flags = 0;
	if (sep == std::string::npos)
		return MsgHeader(0xfff);


	if ((s.size() > sep + 1) && (s[sep + 1] == '#')) {
		// CAN FD frame. First nibble after ## is FD flags.
		is_fd = true;
		if (!hex2dec(fd_flags, s[sep + 2]))
			return MsgHeader(0xfff);
		sep += 2;
	}

	Header header = toheader(s.substr(0, sep));
	header.flags = fd_flags;
	header.is_fd = is_fd;
	Frame frame(header);
	std::string buffer;
	if (header.isValid() && hex2buffer(buffer, s.substr(sep + 1), false)) {
		if (buffer.size() > frame.data.size())
			return MsgHeader(0xfff);

		for (size_t i = 0; i < buffer.size(); ++i) {
			frame.data[i] = buffer[i];
		}
		frame.dlc = buffer.size();
	}
	return frame;
}

template<> FrameFilterSharedPtr tofilter(const std::string  &s){
	  FrameFilter * filter = 0;
		size_t delim = s.find_first_of(":~-_");

		uint32_t second = FrameMaskFilter::MASK_RELAXED;
		bool invert = false;
		char type = ':';

		if(delim != std::string::npos) {
			type = s.at(delim);
			second = tohex(s.substr(delim +1));
		}
		uint32_t first = toheader(s.substr(0, delim)).fullid();
		switch (type) {
			case '~':
				invert = true;
			case ':':
				filter = new FrameMaskFilter(first, second, invert);
				break;
			case '_':
				invert = true;
			case '-':
				filter = new FrameRangeFilter(first, second, invert);
				break;
		}
		return FrameFilterSharedPtr(filter);
}
template<> FrameFilterSharedPtr tofilter(const uint32_t &id){
		return FrameFilterSharedPtr(new FrameMaskFilter(id));
}

FrameFilterSharedPtr tofilter(const char* s){
		return tofilter<std::string>(s);
}

std::ostream& operator <<(std::ostream& stream, const Header& h) {
	return stream << can::tostring(h, true);
}

std::ostream& operator <<(std::ostream& stream, const Frame& f) {
	return stream << can::tostring(f, true);
}

}
