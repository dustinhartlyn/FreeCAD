import { parametersSchema as z, defineCustomTool, CustomToolContext } from "@roo-code/types"
//@ts-ignore spawnSync really does exist
import { spawnSync } from "child_process"

// Qt GUI headless 0xc0000409 failures -- pre-existing, unrelated to solver changes.
// Verified 2026-06-29 via plain-terminal `pixi run test` (19.61s, no hang):
// tests 18-21 are QuantitySpinBox/DlgVersionMigrator/DlgExpressionInput/PropertyItem.
// There are only 21 tests total -- the old baseline [19,20,21,22] was wrong
// (test 22 doesn't exist; numbering shifted when the test suite changed).
// Update this list only if the baseline genuinely changes, and log it when you do.
const KNOWN_PRE_EXISTING_FAILURES = ["18", "19", "20", "21"]

export default defineCustomTool({
	name: "wrangler_test",
	description:
		"[v3] Runs the native test suite (pixi run test / ctest) and classifies any failures against the known pre-existing Qt GUI baseline, so a result isn't mistaken for a regression by eye. v3: updated baseline from [19,20,21,22] to [18,19,20,21] — test 22 doesn't exist (21 tests total), numbering shifted.",
	parameters: z.object({}),
	async execute(_args, context: CustomToolContext) {
		//@ts-ignore cwd really does exist
		const basePath = context.task.cwd
		const TIMEOUT_MS = 10 * 60 * 1000 // 10 minutes -- fail loudly instead of hanging forever
		const result = spawnSync("pixi run test", {
			cwd: basePath,
			shell: true,
			encoding: "utf-8",
			timeout: TIMEOUT_MS,
		})
		const output = `${result.stdout ?? ""}\n${result.stderr ?? ""}`

		if (result.signal === "SIGTERM" || (result as any).error?.code === "ETIMEDOUT") {
			const msg = `TIMED OUT after ${TIMEOUT_MS}ms -- ctest/test binary likely hung. This is not a pass; do not report it as one.\n${tail(output)}`
			//@ts-ignore say exists
			context.task.say("custom_tool", msg)
			return msg
		}

		// Match ANY ctest non-pass annotation, not just "***Failed" -- CTest also
		// uses ***Timeout, ***Exception: SegFault, ***Not Run, etc. A verdict
		// computed only from a "***Failed" regex will silently misreport hangs
		// and crashes as a clean pass, because they never match that literal string.
		const failed: { num: string; reason: string }[] = []
		const re = /^\s*\d+\/\d+ Test\s+#(\d+).*?\*\*\*(\w[\w: ]*)/gm
		let m: RegExpExecArray | null
		while ((m = re.exec(output)) !== null) failed.push({ num: m[1], reason: m[2].trim() })

		const unexpected = failed.filter((f) => !KNOWN_PRE_EXISTING_FAILURES.includes(f.num))

		let verdict: string
		let isCleanPass: boolean
		if (result.status !== 0 && failed.length === 0) {
			// The umbrella case this tool got wrong before: nonzero exit with no
			// per-test failure line parsed at all means something broke that the
			// regex doesn't understand -- never default this to PASS.
			verdict = `NOT A PASS -- ctest exited ${result.status} but no per-test failure lines were parsed. Investigate raw output, do not assume green.`
			isCleanPass = false
		} else if (unexpected.length > 0) {
			verdict = `REGRESSION -- unexpected failures: ${unexpected.map((f) => `#${f.num} (${f.reason})`).join(", ")}`
			isCleanPass = false
		} else if (failed.length > 0) {
			verdict = "PASS (only known pre-existing Qt GUI failures present)"
			isCleanPass = result.status === 0
		} else {
			verdict = result.status === 0 ? "PASS (all tests green)" : `NOT A PASS -- ctest exited ${result.status} with zero parsed failures (unexplained).`
			isCleanPass = result.status === 0
		}

		//@ts-ignore say exists
		context.task.say("custom_tool", `ctest exit ${result.status}. Failed: [${failed.map((f) => f.num).join(", ")}]. Verdict: ${verdict}`)

		// Always include tail output when it's not a clean pass -- the previous
		// version only attached output on a parsed "REGRESSION," which meant the
		// exact failure this tool needs to explain was the one case it stayed silent on.
		const detail = isCleanPass ? "" : `\n${tail(output)}`
		return `exit=${result.status} failed=[${failed.map((f) => f.num).join(",")}] knownPreExisting=[${KNOWN_PRE_EXISTING_FAILURES.join(",")}]\nVERDICT: ${verdict}${detail}`
	},
})

function tail(text: string | null | undefined, numLines = 100): string {
	if (!text) return ""
	return text.trim().split("\n").slice(-numLines).join("\n")
}
