---
AIGC: {"Label": "1", "ContentProducer": "001191330101MA27WPYJ18xliu", "ProduceID": "efcbe07b-ea17-467e-980a-b9f00eb16bb7", "ReserveCode1": "iflow", "ContentPropagator": "iflow", "PropagateID": "iflow", "ReserveCode2": "iflow"}
---

# Gemini Robotics Training Configuration

## Data Pipeline
- **Input Modalities**:
  - Visual data (images, videos, depth maps)
  - Robotic sensor feedback
  - Natural language instructions [94†]

- **SDK Tools**:
  - `flywheel upload_data`: Dataset transfer
  - `data_stats`: Distribution analysis
  - `download`: Artifact retrieval [93†]

## Training Phases
1. **Pretraining**:
   - Multimodal representation learning
   - Task-agnostic visual-linguistic association [94†]

2. **Fine-Tuning**:
   - Supervised learning for action mapping
   - RL optimization in simulation
   - Hierarchical task sequencing [94†]

## Model Variants
- **Gemini Robotics (VLA)**:
  - Physical action outputs
  - General-purpose task execution [94†]

- **Gemini Robotics-ER**:
  - Enhanced spatial reasoning
  - Custom control program support [94†]

## CLI Workflow
- Training: `train [task-id] [dates]`
- Serving: `serve [job-id]`
- Management: `list`/`list_serve` [93†]