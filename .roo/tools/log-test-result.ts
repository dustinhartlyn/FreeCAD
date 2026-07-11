import { parametersSchema as z, defineCustomTool, CustomToolContext } from "@roo-code/types"
//@ts-ignore fs really does exist
import * as fs from "fs"
//@ts-ignore path really does exist
import * as path from "path"

export default defineCustomTool({
	name: "log_test_result",
	description:
		"[v1] Appends a structured entry to the Test & Benchmark Results Log in Agent_Notes/.gcs_status.md. This is the ONLY accepted way to satisfy the 'a test was run' logging requirement -- do not hand-write this entry in prose instead.",
	parameters: z.object({
		title: z.string().describe("Short title, e.g. 'Stage 4: Cluster Decomposition Differential Test (shared-point case)'"),
		command: z.string().describe("The exact command that was run"),
		hypothesis: z
			.string()
			.describe(
				"The SPECIFIC, narrow claim under test. If the test only covers a subset of a larger plan's claim, state that narrowing explicitly here -- do not let a narrow result get logged in a way that reads as confirming the general case."
			),
		expected: z.string().describe("What the plan predicted BEFORE the actual result was seen"),
		actual: z.string().describe("The actual output, verbatim -- pass/fail counts, numbers, assertion failures"),
		verdict: z
			.enum(["CONFIRMED", "FALSIFIED", "INCONCLUSIVE"])
			.describe("No other value accepted -- this stops 'PASSED (mostly)' from sliding in"),
	}),
	async execute({ title, command, hypothesis, expected, actual, verdict }, context: CustomToolContext) {
		//@ts-ignore cwd really does exist
		const basePath = context.task.cwd
		const statusPath = path.join(basePath, "Agent_Notes", ".gcs_status.md")
		const date = new Date().toISOString().slice(0, 16).replace("T", " ")

		let existing = ""
		try {
			existing = fs.readFileSync(statusPath, "utf-8")
		} catch {
			// file may not exist yet
		}
		if (!/## .*Test & Benchmark Results Log/.test(existing)) {
			fs.appendFileSync(statusPath, "\n## Test & Benchmark Results Log\n", "utf-8")
		}

		const entry = `
### ${title} (${date})

**Command**: \`${command}\`

**Hypothesis** (state the exact, narrow claim -- not a generalization of it): ${hypothesis}

**Expected** (stated before the result was seen): ${expected}

**Actual** (verbatim): ${actual}

**Verdict**: ${verdict}
`
		fs.appendFileSync(statusPath, entry, "utf-8")
		return `Logged to Agent_Notes/.gcs_status.md: ${title} -> ${verdict}`
	},
})
