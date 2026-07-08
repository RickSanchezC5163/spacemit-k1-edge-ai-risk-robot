# Local LLM Report Interface - 2026-07-01

Unified function:

```python
generate_risk_report(
    episode_report_path: str,
    risk_map_summary_path: str | None,
    backend: str = "deterministic",
    output_dir: str | None = None,
) -> dict
```

Backends:

- `deterministic`
- `llama_cpp_qwen_0_5b`
- `llama_cpp_tinyllama`
- `stub_local_llm`

Outputs:

- `risk_control_report.md`
- `risk_control_report.json`
- `llm_benchmark.json`
- `prompt.txt`
- `raw_output.txt`
- `claim_boundary.md`

Benchmark fields:

- TTFT
- tokens/s
- total tokens
- peak memory
- model name
- model size

Current status: deterministic report is stable, but it is not a real LLM. A
future llama.cpp backend must run locally and record benchmark data before any
local LLM claim.
