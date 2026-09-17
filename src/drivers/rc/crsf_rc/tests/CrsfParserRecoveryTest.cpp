#include "CrsfParser.hpp"
#include "Crc8.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
using Bytes = std::vector<uint8_t>;
CrsfParserStatistics_t stats{};
CrsfPacket_t last{};
unsigned channels = 0;
unsigned links = 0;

void check(bool condition, const char *message)
{
	if (!condition) { throw std::runtime_error(message); }
}

void reset()
{
	CrsfParser_Init();
	stats = {};
	last = {};
	channels = links = 0;
}

void drain()
{
	CrsfPacket_t packet{};
	unsigned iterations = 0;
	while (CrsfParser_TryParseCrsfPacket(&packet, &stats)) {
		check(++iterations < 1000, "parser did not drain");
		last = packet;
		if (packet.message_type == CRSF_MESSAGE_TYPE_RC_CHANNELS) { ++channels; }
		if (packet.message_type == CRSF_MESSAGE_TYPE_LINK_STATISTICS) { ++links; }
	}
}

void feed(const uint8_t *data, size_t size)
{
#ifdef CRSF_BASELINE_TEST
	check(CrsfParser_LoadBuffer(data, size), "baseline receive queue stopped accepting bytes");
#else
	check(CrsfParser_LoadBufferWithRecovery(data, size, &stats), "receive queue rejected normal UART chunk");
#endif
	drain();
}

void feed(const Bytes &bytes, size_t chunk = 64)
{
	for (size_t offset = 0; offset < bytes.size(); offset += chunk) {
		feed(bytes.data() + offset, std::min(chunk, bytes.size() - offset));
	}
}

Bytes frame(uint8_t type, const Bytes &payload)
{
	Bytes bytes{0xc8, static_cast<uint8_t>(payload.size() + 2), type};
	bytes.insert(bytes.end(), payload.begin(), payload.end());
	bytes.push_back(Crc8Calc(bytes.data() + 2, payload.size() + 1));
	return bytes;
}

Bytes rc_frame()
{
	Bytes payload(22, 0);
	for (unsigned channel = 0; channel < 16; ++channel) {
		const unsigned value = 172 + channel * 100;
		for (unsigned bit = 0; bit < 11; ++bit) {
			const unsigned pos = channel * 11 + bit;
			payload[pos / 8] |= ((value >> bit) & 1) << (pos % 8);
		}
	}
	return frame(0x16, payload);
}

void verify_channels()
{
	check(last.message_type == CRSF_MESSAGE_TYPE_RC_CHANNELS, "not an RC packet");
	for (unsigned channel = 0; channel < 16; ++channel) {
		const float expected = 1000.f + (channel * 100.f / (1811.f - 172.f)) * 1000.f;
		check(std::fabs(last.channel_data.channels[channel] - expected) < .001f, "channel decode changed");
	}
}

void normal()
{
	const auto rc = rc_frame();
	for (size_t chunk = 1; chunk <= 64; ++chunk) {
		reset();
		Bytes burst;
		for (unsigned n = 0; n < 100; ++n) { burst.insert(burst.end(), rc.begin(), rc.end()); }
		feed(burst, chunk);
		check(channels == 100 && stats.crcs_invalid == 0, "fragmented or wrapped stream lost channels");
		verify_channels();
	}
	reset();
	const Bytes payload{45, 46, 99, 7, 0, 2, 3, 47, 98, 6};
	feed(frame(0x14, payload), 1);
	check(links == 1 && channels == 0, "link stats mistaken for control data");
	check(last.link_statistics.uplink_link_quality == 99 && last.link_statistics.uplink_rssi_1 == 45,
	      "link statistics changed");
}

void lengths()
{
	const auto rc = rc_frame();
	for (unsigned length = 0; length <= 255; ++length) {
		for (uint8_t type : {uint8_t{0x7f}, uint8_t{0x16}, uint8_t{0x14}}) {
			reset();
			feed(Bytes{0xc8, static_cast<uint8_t>(length), type});
			for (unsigned n = 0; n < 10; ++n) { feed(rc); }
			check(channels > 0, "length corruption blocked subsequent valid packets");
			verify_channels();
		}
	}
	for (uint8_t length : {uint8_t{0}, uint8_t{1}}) {
		reset();
		feed(rc);
		feed(Bytes{0xc8, length, 0x7f});
		for (unsigned n = 0; n < 1000; ++n) { feed(rc); }
		check(channels == 1001, "reported 0/1-length stall did not recover");
		check(stats.invalid_unknown_packet_sizes == 1, "bad short length not counted");
	}
}

void restart()
{
	const auto rc = rc_frame();
	reset();
	feed(Bytes{0xc8, 0, 0x7f});
	CrsfParser_Init();
	feed(rc);
	check(channels == 1, "init retained corrupt length state");
	verify_channels();
	for (size_t cut = 1; cut < rc.size(); ++cut) {
		reset();
		feed(rc.data(), cut);
		CrsfParser_Init();
		feed(rc);
		check(channels == 1, "init retained partial frame state");
		verify_channels();
	}
}

void crc()
{
	const auto rc = rc_frame();
	reset();
	Bytes bad = rc;
	bad.back() ^= 1;
	feed(bad);
	check(channels == 0 && links == 0 && stats.crcs_invalid > 0, "bad CRC accepted as live input");
	feed(rc);
	check(channels == 1, "valid packet after bad CRC not recovered");
	for (size_t pos = 0; pos < rc.size(); ++pos) {
		for (unsigned bit = 0; bit < 8; ++bit) {
			reset();
			bad = rc;
			bad[pos] ^= 1u << bit;
			feed(bad);
			check(channels == 0, "single-bit damaged RC packet accepted");
			for (unsigned n = 0; n < 10; ++n) { feed(rc); }
			check(channels > 0, "single-bit error caused persistent loss");
			verify_channels();
		}
	}
}

void unknown()
{
	for (size_t size = 0; size <= 60; ++size) {
		reset();
		feed(frame(0x7f, Bytes(size, 0x55)), 1);
		check(channels == 0 && links == 0, "unknown packet generated control data");
		check(stats.crcs_valid_unknown_packets == 1, "valid unknown frame rejected");
		feed(rc_frame());
		check(channels == 1, "unknown frame disrupted next RC frame");
	}
}

void overflow()
{
#ifndef CRSF_BASELINE_TEST
	reset();
	feed(Bytes{0xc8, 24, 0x16});
	Bytes noise(190, 0x55);
	check(CrsfParser_LoadBuffer(noise.data(), noise.size()), "overflow test preload failed");
	feed(rc_frame());
	check(stats.queue_overflow_resets == 1 && channels == 1, "overflow did not reset and retain new data");
	verify_channels();
	Bytes oversized(201, 0x55);
	check(!CrsfParser_LoadBufferWithRecovery(oversized.data(), oversized.size(), &stats), "oversize chunk accepted");
	check(stats.queue_overflow_resets == 2, "overflow count lost across reset");
	feed(rc_frame());
	check(channels == 2, "parser failed after oversized chunk");
#endif
}

void noise()
{
	reset();
	uint32_t random = 0x68dc2301;
	for (unsigned n = 0; n < 10000; ++n) {
		Bytes bytes(1 + n % 64);
		for (auto &byte : bytes) {
			random = random * 1664525u + 1013904223u;
			byte = random >> 24;
		}
		feed(bytes);
		const unsigned before = channels;
		for (unsigned k = 0; k < 5; ++k) { feed(rc_frame()); }
		check(channels > before, "noise caused nonrecoverable parser state");
		verify_channels();
	}
}
}

int main(int argc, char **argv)
{
	Crc8Init(0xd5);
	if (argc != 2) { std::fprintf(stderr, "usage: %s normal|lengths|restart|crc|unknown|overflow|noise\n", argv[0]); return 2; }
	try {
		if (!std::strcmp(argv[1], "normal")) { normal(); }
		else if (!std::strcmp(argv[1], "lengths")) { lengths(); }
		else if (!std::strcmp(argv[1], "restart")) { restart(); }
		else if (!std::strcmp(argv[1], "crc")) { crc(); }
		else if (!std::strcmp(argv[1], "unknown")) { unknown(); }
		else if (!std::strcmp(argv[1], "overflow")) { overflow(); }
		else if (!std::strcmp(argv[1], "noise")) { noise(); }
		else { return 2; }
		std::printf("%s: PASS\n", argv[1]);
		return 0;
	} catch (const std::exception &error) {
		std::fprintf(stderr, "%s: FAIL: %s\n", argv[1], error.what());
		return 1;
	}
}
