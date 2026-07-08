# Local LLM Research - 2026-06-25

Goal: select a local language model path for risk explanation, handling advice,
and event summaries on or near the K1 platform.

## Candidate Matrix

| Model | Parameters | Quantization | File size | Runtime | K1 build difficulty | Estimated memory | TTFT | Token/s | Chinese explanation | Recommendation |
| --- | ---: | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Qwen2.5-0.5B-Instruct-GGUF | 0.5B | Q4_K_M / Q5_K_M | TBD | llama.cpp | medium | TBD | TBD | TBD | good for size | First priority |
| Qwen2.5-1.5B-Instruct-GGUF | 1.5B | Q4_K_M | TBD | llama.cpp | medium-high | TBD | TBD | TBD | stronger | Enhanced option |
| SmolLM2-360M | 360M | GGUF Q4 | TBD | llama.cpp | medium | TBD | TBD | TBD | weaker Chinese | Backup experiment |
| SmolLM2-1.7B | 1.7B | GGUF Q4 | TBD | llama.cpp | medium-high | TBD | TBD | TBD | acceptable | Backup if Qwen fails |
| Template interpreter | none | none | small | Python rules | low | low | n/a | n/a | stable templates | Mandatory fallback |

## Preferred Path

1. First priority: Qwen2.5-0.5B-Instruct-GGUF + llama.cpp.
2. Enhanced option: Qwen2.5-1.5B-Instruct-GGUF if memory and speed are acceptable.
3. Fallback: template interpreter using `configs/sop_knowledge_base.json`.

## First Test Prompt Shape

Input:

```json
{
  "event_type": "soft_obstacle",
  "risk_level": "medium",
  "distance_m": 0.8,
  "confidence": 0.9,
  "recommended_action": "stop_and_recheck"
}
```

Expected output:

- one-sentence risk explanation
- one handling recommendation
- one safety note
- short event summary for logs

## Open Measurements

- compile time on K1
- model load time
- TTFT
- token/s
- peak memory
- CPU temperature
- whether the model can run while ROS nodes are active
