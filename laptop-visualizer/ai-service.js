'use strict';

const fs = require('fs');
const path = require('path');
const https = require('https');

const CONFIG_PATH = path.join(__dirname, 'config.json');

function loadConfig() {
  try {
    const raw = fs.readFileSync(CONFIG_PATH, 'utf8');
    return JSON.parse(raw);
  } catch (err) {
    throw new Error(`Failed to load config.json: ${err.message}`);
  }
}

function validateConfig(config) {
  if (!config.geminiApiKey || config.geminiApiKey === 'YOUR_GEMINI_API_KEY_HERE') {
    throw new Error('Please set your Gemini API key in config.json');
  }
  if (!config.geminiModel) {
    throw new Error('geminiModel is required in config.json');
  }
  return true;
}

/**
 * Builds a detailed prompt for the Gemini AI with crash scenario context
 * @param {Object} scenario - The crash scenario data
 * @returns {string} The formatted prompt
 */
function buildPrompt(scenario) {
  const {
    car1,
    car2,
    ttc,
    surroundings,
    timestamp
  } = scenario;

  const scenarioDirective = scenario.scenarioId === 'vid2'
    ? `## REQUIRED VID 2 MANEUVER
For this specific Vid 2 demonstration, return these two coordinated actions:
- CAR1 must use SWERVE_LEFT_BRAKE. Its reasoning must say: "CAR1 was already in the right-turn lane, so continuing with the turn is safer than proceeding through the intersection."
- CAR2 must use SWERVE_RIGHT with a small turn rate (approximately 5 degrees per second) and no additional braking.
Do not substitute different actions for this demonstration.`
    : '';

  return `You are an autonomous vehicle collision avoidance AI system. Analyze this crash scenario and provide evasive action decisions.

## CRASH SCENARIO ANALYSIS REQUEST

**Timestamp:** ${timestamp}
**Time Until Collision (TTC):** ${ttc.toFixed(2)} seconds

### VEHICLE 1 (Our Vehicle - CAR1)
- **Position:** Latitude ${car1.latitude.toFixed(6)}, Longitude ${car1.longitude.toFixed(6)}
- **Local Coordinates:** X=${car1.localX.toFixed(2)}m, Y=${car1.localY.toFixed(2)}m
- **Heading:** ${car1.heading.toFixed(1)}° (0=North, 90=East)
- **Speed:** ${car1.speed.toFixed(2)} m/s (${(car1.speed * 3.6).toFixed(1)} km/h)
- **Dimensions:** ${car1.length}m long × ${car1.width}m wide
- **Current Action:** ${car1.action || 'NONE'}

### VEHICLE 2 (Other Vehicle - CAR2)
- **Position:** Latitude ${car2.latitude.toFixed(6)}, Longitude ${car2.longitude.toFixed(6)}
- **Local Coordinates:** X=${car2.localX.toFixed(2)}m, Y=${car2.localY.toFixed(2)}m
- **Heading:** ${car2.heading.toFixed(1)}° (0=North, 90=East)
- **Speed:** ${car2.speed.toFixed(2)} m/s (${(car2.speed * 3.6).toFixed(1)} km/h)
- **Dimensions:** ${car2.length}m long × ${car2.width}m wide
- **Current Action:** ${car2.action || 'NONE'}

### SURROUNDINGS & ENVIRONMENT
${surroundings}

### RELATIVE POSITION ANALYSIS
- **Distance between vehicles:** ${scenario.distance.toFixed(2)}m
- **Relative bearing from Car1 to Car2:** ${scenario.relativeBearing.toFixed(1)}°
- **Closing speed:** ${scenario.closingSpeed.toFixed(2)} m/s

${scenarioDirective}

## YOUR TASK
Based on this scenario, determine coordinated evasive maneuvers for both CAR1 and CAR2 to avoid collision.

**Available Actions:**
1. SWERVE_LEFT - Turn left to avoid
2. SWERVE_RIGHT - Turn right to avoid
3. BRAKE - Apply brakes
4. SWERVE_LEFT_BRAKE - Swerve left while braking
5. SWERVE_RIGHT_BRAKE - Swerve right while braking
6. EMERGENCY_STOP - Maximum braking

**Response Format (JSON only, no other text):**
{
  "car1": {
    "action": "SWERVE_LEFT_BRAKE",
    "turnRate": 15.0,
    "targetSpeed": 5.0,
    "confidence": 0.95,
    "reasoning": "Brief explanation of why this action is optimal for CAR1"
  },
  "car2": {
    "action": "SWERVE_RIGHT",
    "turnRate": 5.0,
    "targetSpeed": 7.5,
    "confidence": 0.90,
    "reasoning": "Brief explanation of why this action is optimal for CAR2"
  }
}

Where:
- car1 and car2 must each contain all five fields shown above
- action: One of the available actions listed above
- turnRate: Degrees per second for steering (0-30, use 0 for pure braking)
- targetSpeed: Target speed in m/s after maneuver (0 for full stop)
- confidence: Your confidence in this decision (0.0-1.0)
- reasoning: Brief explanation of that vehicle's decision

Respond with ONLY the JSON object, no additional text.`;
}

/**
 * Calls the Gemini API with the given prompt
 * @param {string} prompt - The prompt to send
 * @param {Object} config - The configuration object
 * @returns {Promise<Object>} The parsed AI response
 */
function callGeminiApi(prompt, config) {
  return new Promise((resolve, reject) => {
    const url = new URL(
      `${config.geminiApiUrl}/${config.geminiModel}:generateContent?key=${config.geminiApiKey}`
    );

    const requestBody = JSON.stringify({
      contents: [{
        parts: [{ text: prompt }]
      }],
      generationConfig: {
        temperature: 0.2,
        topK: 40,
        topP: 0.95,
        maxOutputTokens: 2048,
        thinkingConfig: {
          thinkingLevel: 'low'
        },
        responseMimeType: 'application/json'
      }
    });

    const options = {
      hostname: url.hostname,
      port: 443,
      path: url.pathname + url.search,
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(requestBody)
      }
    };

    const req = https.request(options, (res) => {
      let data = '';

      res.on('data', (chunk) => {
        data += chunk;
      });

      res.on('end', () => {
        if (res.statusCode !== 200) {
          reject(new Error(`Gemini API error (${res.statusCode}): ${data}`));
          return;
        }

        try {
          const response = JSON.parse(data);
          const textContent = response.candidates?.[0]?.content?.parts?.[0]?.text;

          console.log('[ai-service] Gemini finish reason:', response.candidates?.[0]?.finishReason);
          console.log('[ai-service] Gemini usage:', response.usageMetadata);

          if (!textContent) {
            reject(new Error('No response content from Gemini API'));
            return;
          }

          // Parse the AI's JSON response
          console.log('[ai-service] Raw Gemini response:', textContent);
          const aiResponse = JSON.parse(textContent);
          resolve(aiResponse);
        } catch (err) {
          reject(new Error(`Failed to parse Gemini response: ${err.message}`));
        }
      });
    });

    req.on('error', (err) => {
      reject(new Error(`Gemini API request failed: ${err.message}`));
    });

    req.setTimeout(30000, () => {
      req.destroy();
      reject(new Error('Gemini API request timed out after 30 seconds'));
    });

    req.write(requestBody);
    req.end();
  });
}

/**
 * Main function to get AI decision for a crash scenario
 * @param {Object} scenario - The crash scenario data
 * @returns {Promise<Object>} The AI decision
 */
async function getAiDecision(scenario) {
  const config = loadConfig();
  validateConfig(config);

  const prompt = buildPrompt(scenario);
  console.log('[ai-service] Sending prompt to Gemini API...');
  console.log('[ai-service] TTC:', scenario.ttc.toFixed(2), 'seconds');

  const decision = await callGeminiApi(prompt, config);

  console.log('[ai-service] Received AI decision:', JSON.stringify(decision));
  return decision;
}

/**
 * Validates an AI decision response
 * @param {Object} decision - The AI decision to validate
 * @returns {boolean} True if valid
 */
function validateDecision(decision) {
  const validActions = [
    'SWERVE_LEFT', 'SWERVE_RIGHT', 'BRAKE',
    'SWERVE_LEFT_BRAKE', 'SWERVE_RIGHT_BRAKE', 'EMERGENCY_STOP'
  ];

  function validVehicleDecision(vehicleDecision) {
    if (!vehicleDecision || typeof vehicleDecision !== 'object') return false;
    if (!validActions.includes(vehicleDecision.action)) return false;
    if (typeof vehicleDecision.turnRate !== 'number' || vehicleDecision.turnRate < 0 || vehicleDecision.turnRate > 30) return false;
    if (typeof vehicleDecision.targetSpeed !== 'number' || vehicleDecision.targetSpeed < 0) return false;
    if (typeof vehicleDecision.confidence !== 'number' || vehicleDecision.confidence < 0 || vehicleDecision.confidence > 1) return false;
    if (typeof vehicleDecision.reasoning !== 'string' || vehicleDecision.reasoning.trim().length === 0) return false;
    return true;
  }

  return Boolean(decision && typeof decision === 'object'
    && validVehicleDecision(decision.car1)
    && validVehicleDecision(decision.car2));
}

module.exports = {
  getAiDecision,
  buildPrompt,
  validateDecision,
  loadConfig,
  validateConfig
};
