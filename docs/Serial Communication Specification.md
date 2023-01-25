# Data Format

> Note: `Length` and `Checksum` are in big endian format.

| `Header`              | `Length`                    | `Data`                           | `Checksum`           |
| --------------------- | --------------------------- | -------------------------------- | -------------------- |
| `0x69 0x69 0x20 0x20` | `u32` Length of `Data` body | Data with length `Length`  bytes | `u16` Checksum (see below) |

# Calculating Checksum

Checksum is calculated by adding all the bytes in the data packet into a `u16`, and negating the result

```
checksum = ~(header + length + data) & 0xFFFF
```

# Parsing Flowchart



```mermaid
stateDiagram-v2
	start: Wait for recv
	header: Read header (4 bytes)
	length: Read length (4 bytes)
	data: Read data byte
	increment: Increment bytes_read
	cksm: Read checksum
	verify_cksm: Verify Checksum
	wait: Wait for recv data
	
	state header_done <<choice>>
	state data_done <<choice>>
	state check <<choice>>
	
	[*] --> start
	start --> header_done
	header_done --> header: if recv_buffer_len >= 8
	header_done --> start: else
	header --> length
	length --> wait
	wait --> data
	data --> increment
	increment --> data_done
	data_done --> cksm: if bytes_read == length
	data_done --> wait: else
	cksm --> verify_cksm
	verify_cksm --> check
	check --> [*] : if (checksum is correct)
	check --> start : if (checksum is incorrect)
```